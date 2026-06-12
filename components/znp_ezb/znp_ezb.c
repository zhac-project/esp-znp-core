/*
 * znp_ezb.c — on-target commissioning ops + app signal handler (Task 4.5)
 *
 * Signal handler mechanism: callback registration via ezb_app_signal_add_handler().
 * Called after esp_zigbee_init() in ezb_start_stack(). NOT a weak symbol.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include "znp_ezb.h"
#include "znp_dispatch.h"   /* znp_build_* builders + znp_netcfg_t */
#include "znp_uart.h"       /* znp_uart_send_raw */

#include "esp_mac.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* esp-zigbee-lib v2.0.1 native API */
#include "esp_zigbee.h"             /* esp_zigbee_init/start/launch_mainloop, esp_zigbee_lock_acquire/release, esp_zigbee_config_t */
#include "ezbee/app_signals.h"      /* ezb_app_signal_*, EZB_BDB_SIGNAL_*, EZB_ZDO_SIGNAL_* */
#include "ezbee/af.h"               /* ezb_af_create_endpoint_desc, ezb_af_create_device_desc, etc. */
#include "ezbee/bdb.h"              /* ezb_bdb_start_top_level_commissioning, ezb_bdb_open_network */
#include "ezbee/core.h"             /* ezb_set_panid, ezb_set_use_extended_panid, ezb_set_channel_mask, ezb_get_panid, ezb_get_current_channel */
#include "ezbee/core_types.h"       /* ezb_panid_t, ezb_extpanid_t, ezb_extaddr_t (=ezb_eui64_s) */
#include "ezbee/nwk.h"              /* ezb_nwk_set_device_type, ezb_nwk_get_short_address, ezb_nwk_get_panid */
#include "ezbee/secur.h"            /* ezb_secur_set_network_key */
#include "ezbee/error.h"            /* ezb_err_t; success == 0 (no 0 constant) */

#include <string.h>

static const char *TAG = "znp_ezb";

/* ── Network-up flag, set from signal handler task context ──────────────── */
static volatile bool s_net_up = false;

/* F32 (FINDINGS.md): ezb_get_nwk_info() runs on the UART RX task and previously
 * read ezb_nwk_get_panid()/short_address() — stack-internal state — without the
 * stack lock, racing the mainloop. Instead we snapshot the identity here, in the
 * signal-handler (mainloop) context, whenever the network comes up, and serve
 * that cached copy to the RX task. 16-bit aligned loads/stores are atomic on the
 * RISC-V target, so no lock is needed for the reader. */
static volatile uint16_t s_cached_panid = 0xFFFFu;
static volatile uint16_t s_cached_short = 0xFFFEu;

/* called only from the signal handler (callback ctx) → intentionally lockless;
 * see LOCK DISCIPLINE. */
static void cache_nwk_info(void)
{
    s_cached_panid = (uint16_t)ezb_nwk_get_panid();
    s_cached_short = (uint16_t)ezb_nwk_get_short_address();
}

/* T36 / FINDINGS §12 MED (def 3): invalidate the cached identity when the
 * network goes down (self-leave / steering-fail / NWK failure). Reset to the
 * "no network" sentinels so a host ZDO_EXT_NWK_INFO poll after the network dies
 * reads a coherent down state, not a stale panid/short from the dead network.
 * Lockless — only ever called from the signal handler (callback ctx). */
static void invalidate_nwk_info(void)
{
    s_cached_panid = 0xFFFFu;
    s_cached_short = 0xFFFEu;
}

/* T36 / FINDINGS §12 MED (def 2 + def 3): bring the network up/down as ONE
 * ordered operation. On up: cache the identity FIRST, then publish s_net_up —
 * so a concurrent EXT_NWK_INFO reader never observes dev_state=0x09 paired with
 * a stale panid 0xFFFF. On down: clear s_net_up FIRST, then invalidate the
 * cache — so a reader never observes dev_state=0x09 paired with a wiped panid.
 * Both run lockless in the signal handler (callback ctx); the 16-bit + bool
 * stores are individually atomic on RISC-V, and the ordering closes the window
 * the reader cares about (it gates the panid read on dev_state==0x09). */
static void set_net_up(void)
{
    cache_nwk_info();
    s_net_up = true;
}

static void set_net_down(void)
{
    s_net_up = false;
    invalidate_nwk_info();
}

/* ── IEEE / reset (unchanged from stub) ─────────────────────────────────── */

static void ezb_get_ieee(uint8_t out[8])
{
    uint8_t mac[8] = {0};
    /* 802.15.4 EUI64 from efuse. esp_read_mac returns it MSB-first; the MT
     * ExtAddr field is little-endian, so reverse. VERIFY orientation on first
     * P4 hardware bring-up (compare P4 log vs chip label). */
    esp_err_t err = esp_read_mac(mac, ESP_MAC_IEEE802154);
    if (err == ESP_OK) {
        for (int i = 0; i < 8; i++) out[i] = mac[7 - i];
    } else {
        /* T36 / FINDINGS §12 LOW (def 5): an all-zero IEEE is an invalid EUI64
         * the host may accept as the coordinator address — surface the read
         * failure instead of silently serving zeros. */
        ESP_LOGE(TAG, "esp_read_mac(IEEE802154) failed: %d — serving all-zero IEEE", err);
        memset(out, 0, 8);
    }
}

static void ezb_request_reset(void)
{
    esp_restart();   /* reboots → app_main emits SYS_RESET_IND on next boot */
}

/* ── Factory-reset latch (def 1 / FINDINGS §12 CRIT) ──────────────────────── *
 * The host writes NV STARTUP_OPTION with a clear bit set (then SYS_RESET) to
 * demand a blank coordinator. esp_restart() preserves the zb_storage NVRAM, so
 * unless we intervene the old PAN/network-key resurrects against the new config
 * and the radio can never be factory-reset. We persist a one-shot flag in the
 * default `nvs` partition here (on the UART RX task, during NV-write dispatch),
 * then consume it on the NEXT boot inside ezb_start_stack — BEFORE
 * esp_zigbee_init — by erasing the zb_storage partition. */
#define ZNP_FN_NVS_NAMESPACE  "znp"
#define ZNP_FN_NVS_KEY        "factory_new"   /* u8: 1 = erase zb_storage at next bring-up */
#define ZNP_ZB_NVRAM_PARTITION "zb_storage"   /* matches partitions.csv + app_main init */

static void ezb_request_factory_new(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ZNP_FN_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "factory-new latch: nvs_open failed: %d", err);
        return;
    }
    err = nvs_set_u8(h, ZNP_FN_NVS_KEY, 1);
    if (err == ESP_OK) err = nvs_commit(h);   /* must survive the imminent reset */
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "factory-new latch: persist failed: %d", err);
    } else {
        ESP_LOGW(TAG, "factory-new latch set — zb_storage will be erased on next boot");
    }
}

/* Returns true if the factory-new flag is set, and clears it (one-shot). */
static bool consume_factory_new_flag(void)
{
    nvs_handle_t h;
    if (nvs_open(ZNP_FN_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    uint8_t flag = 0;
    esp_err_t err = nvs_get_u8(h, ZNP_FN_NVS_KEY, &flag);
    bool requested = (err == ESP_OK && flag != 0);
    if (requested) {
        /* Clear the latch BEFORE erasing so a crash mid-erase can't loop the
         * device into a permanent wipe; the host re-requests if needed. */
        nvs_erase_key(h, ZNP_FN_NVS_KEY);
        nvs_commit(h);
    }
    nvs_close(h);
    return requested;
}

/* ── 1. apply_config ─────────────────────────────────────────────────────── *
 * BUG FIX (post Phase-4 HW test): the ezb_set_ and ezb_secur_set_ calls
 * touch stack-internal state that does not exist until esp_zigbee_init()
 * has run.  Calling them first crashed the chip and triggered a reboot.
 *
 * apply_config now CACHES the buffered config; start_stack applies it after
 * esp_zigbee_init() has succeeded.
 */

static znp_netcfg_t s_pending_cfg;
static bool         s_have_pending_cfg = false;

/* ── def 4 / FINDINGS §12 HIGH — per-step init latches ───────────────────── *
 * The old single `s_stack_inited` flag was only set true at the very END of
 * bring-up. Any failure after esp_zigbee_init (handler add / AF reg / start /
 * task create) left it false, so a host STARTUP_FROM_APP retry re-ran
 * esp_zigbee_init on an already-initialised stack and re-registered the signal
 * handler → double-init UB + duplicate AREQ emission per signal. We now latch
 * each step independently so a retry RESUMES from the first incomplete step
 * rather than restarting from scratch. All written/read only on the worker task
 * (do_bring_up) except s_worker_started, which is the RX-task → worker handoff
 * guard (see ezb_start_stack). */
static bool s_zb_inited      = false;   /* esp_zigbee_init done                */
static bool s_handler_added  = false;   /* ezb_app_signal_add_handler done     */
static bool s_af_registered  = false;   /* AF endpoint descriptor registered   */
static bool s_zb_started     = false;   /* esp_zigbee_start(false) done        */
static volatile bool s_worker_started = false;  /* bring-up worker task spawned */
static volatile bool s_bring_up_ok    = false;  /* worker reached launch_mainloop */

static void ezb_apply_config(const znp_netcfg_t *cfg)
{
    if (!cfg) return;

    /* T36 / FINDINGS §12 MED (def 3b): once the stack has started, the buffered
     * config has ALREADY been consumed in bring_up_steps (step 3, gated on
     * s_af_registered) and start_stack is idempotent-true thereafter. Re-staging
     * here would silently swallow the new config — the host's NV writes get an
     * ACK but never take effect, with no path to apply them. The ezb_set_*
     * setters only apply pre-init/pre-start, so a live post-start re-config is
     * not honestly applicable in-place; it requires a reset to re-run bring-up.
     *
     * Honest path: do NOT pretend to stage it. Log loudly that the post-start
     * config will not take effect until a reset (the host's STARTUP_OPTION-clear
     * + SYS_RESET factory-new flow, T33, is the supported way to re-key/re-PAN a
     * running coordinator). The NV-write SRSP already reported acceptance into
     * the cfg buffer for the protocol layer; this log is the operator-visible
     * signal that a running-stack reconfigure needs a reboot to land. */
    if (s_zb_started) {
        ESP_LOGW(TAG, "apply_config after stack start: buffered net config will NOT "
                      "take effect until reset (host must STARTUP_OPTION-clear + reset to re-key/re-PAN)");
        return;   /* do not overwrite the already-consumed s_pending_cfg */
    }

    s_pending_cfg      = *cfg;
    s_have_pending_cfg = true;
}

/* ── Signal handler — forward-declared for use in the bring-up worker ────── */
static bool s_signal_handler(const ezb_app_signal_t *signal);

/* T36 / FINDINGS §12 LOW (def 5): the STATE_CHANGE_IND AREQ is state-bearing —
 * the host's commissioning gate polls for state=9 within a 10 s window (per
 * T35's finding). A single dropped frame (200 ms TX-mutex-timeout) silently
 * stalls that gate with no host retry. znp_uart_send_raw already logs the drop,
 * but a one-shot send has no recovery, so retry a few times before giving up,
 * and LOG loudly if every attempt fails (vs. the previous silent ignore of the
 * return value). Runs in the signal-handler (callback) ctx; the short bounded
 * retries are acceptable there — the mainloop is not latency-critical at the
 * moment the network transitions up. */
#define ZNP_AREQ_RETRY_ATTEMPTS  3
static void send_state_change_ind(uint8_t dev_state)
{
    uint8_t buf[32];
    size_t  n = znp_build_state_change_ind(dev_state, buf, sizeof(buf));
    if (!n) {
        ESP_LOGE(TAG, "state_change_ind(state=0x%02x): build failed", dev_state);
        return;
    }
    for (int attempt = 1; attempt <= ZNP_AREQ_RETRY_ATTEMPTS; attempt++) {
        if (znp_uart_send_raw(buf, n)) {
            return;   /* sent */
        }
        ESP_LOGW(TAG, "state_change_ind(state=0x%02x): send attempt %d/%d failed",
                 dev_state, attempt, ZNP_AREQ_RETRY_ATTEMPTS);
    }
    /* All retries exhausted: a state=9 the host never sees hangs its 10 s
     * commissioning gate. Make the failure LOUD so it shows in the log instead
     * of a silent stall. The host's own state-timeout → STARTUP retry remains
     * the upper-layer recovery. */
    ESP_LOGE(TAG, "state_change_ind(state=0x%02x): DROPPED after %d attempts — "
                  "host commissioning gate may stall", dev_state, ZNP_AREQ_RETRY_ATTEMPTS);
}

/* ── Stack bring-up — runs on the dedicated worker task, NOT the RX task ──── *
 * def 3 / FINDINGS §12 HIGH: the entire ZBOSS bring-up (NVRAM erase, init, AF
 * registration, start) used to run on the 4 KB TWDT-subscribed UART RX task. A
 * slow flash op there could trip the 5 s TWDT panic mid-commissioning, and any
 * long handler stalled MT-frame parsing + SRSP timeliness. Bring-up now lives
 * on this worker; the RX loop only parses → dispatches-light → replies. The
 * worker is deliberately NOT TWDT-subscribed: after init it blocks forever in
 * esp_zigbee_launch_mainloop(), which would otherwise look like a wedge. The
 * accepted cost is that a wedged bring-up BEFORE launch_mainloop is WDT-unwatched
 * — recovery relies on the host's state-timeout → retry, not a chip hang.
 *
 * Per-step latches (def 4) make each step resume-not-restart on a retry. */
static bool bring_up_steps(void)
{
    /* 0. Factory-reset consume (T33 / FINDINGS §12 CRIT).
     * !!! MUST run before esp_zigbee_init — DO NOT move below it. !!!
     * P6-T35 relocated this bring-up off the UART RX task onto this worker; the
     * erase STAYS unconditionally BEFORE esp_zigbee_init: the stack reads the
     * zb_storage NVRAM during init, so wiping it afterward would not give the
     * host the blank radio it asked for via STARTUP_OPTION. esp_restart()
     * preserves zb_storage; this is the only point the prior PAN/key can clear.
     * Gated on !s_zb_inited so a post-init retry doesn't re-erase a live NVRAM. */
    if (!s_zb_inited && consume_factory_new_flag()) {
        /* app_main already nvs_flash_init_partition("zb_storage")'d this
         * partition, so erase + re-init here operates on an already-mounted,
         * active partition — that is safe (erase deinits then re-formats it). */
        esp_err_t eerr = nvs_flash_erase_partition(ZNP_ZB_NVRAM_PARTITION);
        if (eerr != ESP_OK) {
            /* CRIT: a failed erase means the prior PAN/network-key survives. We
             * MUST NOT proceed into esp_zigbee_init over the stale partition —
             * that silently resurrects the old network the host asked us to
             * wipe. The latch was already consumed (cleared) above, so re-set
             * it and reboot: the next boot retries the wipe rather than coming
             * up on stale state. */
            ESP_LOGE(TAG, "factory-new: erase %s FAILED: %d — re-arming latch + restart",
                     ZNP_ZB_NVRAM_PARTITION, eerr);
            ezb_request_factory_new();
            esp_restart();   /* no return */
        }
        ESP_LOGW(TAG, "factory-new: erased %s partition before stack init",
                 ZNP_ZB_NVRAM_PARTITION);
        /* Re-init the partition so esp_zigbee_init sees a clean, mounted NVS.
         * If this fails the stack would run against an unmounted partition —
         * equally unsafe, so re-arm + restart on the same fail-safe path. */
        esp_err_t ierr = nvs_flash_init_partition(ZNP_ZB_NVRAM_PARTITION);
        if (ierr != ESP_OK) {
            ESP_LOGE(TAG, "factory-new: re-init %s FAILED: %d — re-arming latch + restart",
                     ZNP_ZB_NVRAM_PARTITION, ierr);
            ezb_request_factory_new();
            esp_restart();   /* no return */
        }
    }

    /* 1. Stack init (must run BEFORE any ezb_set_*) — latched against re-init. */
    if (!s_zb_inited) {
        esp_zigbee_config_t cfg = {
            .device_config = {
                .device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR,
            },
        };
        esp_err_t err = esp_zigbee_init(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_zigbee_init failed: %d", err);
            return false;
        }
        s_zb_inited = true;
    }

    /* 2. Signal handler — register once (re-registering would double-emit every
     *    AREQ per signal). Latched. */
    if (!s_handler_added) {
        ezb_err_t herr = ezb_app_signal_add_handler(s_signal_handler);
        if (herr != 0) {
            ESP_LOGE(TAG, "ezb_app_signal_add_handler failed: %d", herr);
            return false;
        }
        s_handler_added = true;
    }

    /* 3. Apply the buffered network parameters AFTER init.
     *    These setters mutate stack-internal state; on this worker the mainloop
     *    is not yet running (launch happens in step 6), but the lib task may
     *    already be schedulable, so hold the Zigbee lock defensively per the
     *    esp_zigbee.h:175 cross-task mandate. AF registration (step 4) is
     *    likewise wrapped. */
    if (!s_af_registered) {
        if (!esp_zigbee_lock_acquire(portMAX_DELAY)) {
            ESP_LOGE(TAG, "bring-up: lock acquire failed (config/AF)");
            return false;
        }
        if (s_have_pending_cfg) {
            const znp_netcfg_t *c = &s_pending_cfg;
            if (c->have_pan_id) {
                ezb_set_panid((ezb_panid_t)c->pan_id);
            }
            if (c->have_ext_pan_id) {
                ezb_extpanid_t epid;
                memcpy(epid.u8, c->ext_pan_id, 8);
                ezb_set_use_extended_panid(&epid);
            }
            if (c->have_chan_mask) {
                ezb_set_channel_mask(c->chan_mask);
            }
            if (c->have_nwk_key) {
                ezb_secur_set_network_key(c->nwk_key);
                /* T36 / FINDINGS §12 MED (def 4): the network key is security-
                 * sensitive — once the stack has consumed it, zeroize our staged
                 * copy so the cleartext key does not linger in RAM for the
                 * process lifetime (open-source release: don't leave keys in
                 * memory). Clear the have-flag too so a resume/retry doesn't
                 * re-push a wiped (all-zero) key over the live one. NOTE: this is
                 * the worker's s_pending_cfg copy; app_main's znp_dispatch_ctx
                 * copy (s_ctx.cfg) is zeroized at its own apply site (app_main.c). */
                memset(s_pending_cfg.nwk_key, 0, sizeof(s_pending_cfg.nwk_key));
                s_pending_cfg.have_nwk_key = false;
            }
        }
        /* Always force coordinator role */
        ezb_nwk_set_device_type(EZB_NWK_DEVICE_TYPE_COORDINATOR);

        /* 4. Minimal coordinator endpoint (ep 1, HA profile, device id 0x0000) */
        ezb_af_ep_config_t ep_cfg = {
            .ep_id              = 1,
            .app_profile_id     = EZB_AF_HA_PROFILE_ID,
            .app_device_id      = 0x0000,
            .app_device_version = 0,
        };
        ezb_af_ep_desc_t  ep_desc  = ezb_af_create_endpoint_desc(&ep_cfg);
        ezb_af_device_desc_t dev   = ezb_af_create_device_desc();
        if (!ep_desc || !dev) {
            if (ep_desc) ezb_af_free_endpoint_desc(ep_desc);
            if (dev)     ezb_af_free_device_desc(dev);
            esp_zigbee_lock_release();
            ESP_LOGE(TAG, "AF descriptor alloc failed");
            return false;
        }
        ezb_af_device_add_endpoint_desc(dev, ep_desc);  /* dev owns ep_desc after this */
        ezb_err_t reg_err = ezb_af_device_desc_register(dev);
        if (reg_err != 0) {
            ezb_af_free_device_desc(dev);
            esp_zigbee_lock_release();
            ESP_LOGE(TAG, "ezb_af_device_desc_register failed: %d", reg_err);
            return false;
        }
        esp_zigbee_lock_release();
        s_af_registered = true;
    }

    /* 5. autostart=false — host drives BDB via BDB_START_COMMISSIONING. Latched. */
    if (!s_zb_started) {
        if (!esp_zigbee_lock_acquire(portMAX_DELAY)) {
            ESP_LOGE(TAG, "bring-up: lock acquire failed (start)");
            return false;
        }
        esp_err_t err = esp_zigbee_start(false);
        esp_zigbee_lock_release();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_zigbee_start failed: %d", err);
            return false;
        }
        s_zb_started = true;
    }

    return true;
}

/* Bring-up + mainloop worker task. Owns the long-running stack work so the RX
 * task (which is TWDT-subscribed) never blocks on flash/init. After a clean
 * bring-up it enters esp_zigbee_launch_mainloop(), which never returns under
 * normal operation. NOT TWDT-subscribed (it legitimately blocks forever); the
 * accepted cost is that a wedged bring-up before launch_mainloop is WDT-unwatched,
 * recovered via the host's state-timeout → retry rather than a chip hang. */
static void ezb_worker_task(void *arg)
{
    (void)arg;
    if (!bring_up_steps()) {
        /* A step failed; the per-step latches recorded how far we got. Leave
         * s_bring_up_ok false and exit so a host STARTUP_FROM_APP retry can
         * re-spawn the worker and RESUME from the failed step. */
        ESP_LOGE(TAG, "stack bring-up failed — worker exiting, retry will resume");
        s_worker_started = false;   /* allow re-spawn on retry */
        vTaskDelete(NULL);
        return;
    }
    s_bring_up_ok = true;
    ESP_LOGI(TAG, "stack bring-up complete — entering mainloop");
    /* T36 / FINDINGS §12 LOW (def 5): launch_mainloop returns esp_err_t and
     * normally never returns. If it DOES return, the stack has shut down and the
     * NCP is silently radio-dead (network goes down with the loop). Mark the
     * network down (def 3) and LOG the err before self-deleting, so the failure
     * shows in the log instead of a silent task exit. */
    esp_err_t loop_err = esp_zigbee_launch_mainloop();   /* blocks until shutdown (never in normal op) */
    set_net_down();
    ESP_LOGE(TAG, "esp_zigbee_launch_mainloop returned %d — mainloop exited, NCP radio-dead",
             loop_err);
    s_bring_up_ok = false;
    vTaskDelete(NULL);
}

/* ── 2. start_stack ──────────────────────────────────────────────────────── *
 * Runs on the UART RX task (0x40 ZDO_STARTUP_FROM_APP dispatch). def 3: it no
 * longer performs the bring-up itself — it spawns ezb_worker_task once and
 * returns IMMEDIATELY so the 0x40 SRSP goes out promptly (the host's
 * znp_sreq_retry waits ≤3 s × 3 for the SRSP, then separately polls for the
 * async ZDO_STATE_CHANGE_IND state=9 emitted later by the signal handler — so
 * an immediate-accept SRSP + async readiness is exactly what the host expects;
 * verified in zhac-components zigbee_mgr.cpp coordinator_start). Returning true
 * here means "accepted/started", not "network up". */
static bool ezb_start_stack(void)
{
    /* Idempotent: once the worker is spawned and the bring-up succeeded, a
     * resent STARTUP_FROM_APP is a no-op success. While the worker is still
     * running its first pass, also no-op (don't double-spawn). */
    if (s_worker_started) return true;

    s_worker_started = true;   /* claim the spawn before creating the task */
    BaseType_t rc = xTaskCreatePinnedToCore(
        ezb_worker_task, "ezb_main", 8192, NULL, 5, NULL, 0);
    if (rc != pdPASS) {
        /* def 3: check the create return (was previously ignored elsewhere).
         * Clear the guard so the host can retry the spawn. */
        s_worker_started = false;
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore(ezb_main) failed: %d", (int)rc);
        return false;
    }
    return true;   /* accepted — bring-up proceeds asynchronously on the worker */
}

/* ── 3. bdb_commission ───────────────────────────────────────────────────── */

static bool ezb_bdb_commission(uint8_t ezb_mode)
{
    /* def 1 / FINDINGS §12 CRIT: this runs on the UART RX task while the
     * esp-zigbee mainloop task is live — a cross-task esp_zb_* call. The lib
     * header (esp_zigbee.h:175) makes holding the Zigbee lock MANDATORY for any
     * SDK API invoked OUTSIDE a stack callback. Without it the BDB scheduler is
     * mutated concurrently with the mainloop → data race. Wrap the call. */
    if (!esp_zigbee_lock_acquire(portMAX_DELAY)) {
        ESP_LOGE(TAG, "bdb_commission: lock acquire failed");
        return false;
    }
    ezb_err_t err = ezb_bdb_start_top_level_commissioning(
        (ezb_bdb_comm_mode_mask_t)ezb_mode);
    esp_zigbee_lock_release();
    if (err != 0) {
        ESP_LOGE(TAG, "ezb_bdb_start_top_level_commissioning(%02x) err %d",
                 ezb_mode, err);
        return false;
    }
    return true;
}

/* ── 4. get_nwk_info ─────────────────────────────────────────────────────── */

static bool ezb_get_nwk_info(uint16_t *panid, uint16_t *short_addr,
                              uint8_t *dev_state)
{
    /* F32: return the mainloop-cached snapshot — no cross-task stack read. */
    if (panid)      *panid      = s_cached_panid;
    if (short_addr) *short_addr = s_cached_short;
    /* 0x09 = TI DEV_ZB_COORD once network is up; 0x00 = not started */
    if (dev_state)  *dev_state  = s_net_up ? 0x09u : 0x00u;
    return true;
}

/* ── 5. permit_join ──────────────────────────────────────────────────────── */

static bool ezb_permit_join(uint8_t dur)
{
    /* def 2 / FINDINGS §12 HIGH: same cross-task class as bdb_commission. Now
     * LIVE (T34 wired 0x36 → permit_join → here), reached from the RX task while
     * the mainloop runs. Hold the Zigbee lock (esp_zigbee.h:175 mandate). */
    if (!esp_zigbee_lock_acquire(portMAX_DELAY)) {
        ESP_LOGE(TAG, "permit_join: lock acquire failed");
        return false;
    }
    ezb_err_t err = ezb_bdb_open_network(dur);
    esp_zigbee_lock_release();
    if (err != 0) {
        ESP_LOGE(TAG, "ezb_bdb_open_network(%u) err %d", dur, err);
        return false;
    }
    return true;
}

/* ── App signal handler ──────────────────────────────────────────────────── */
/*
 * Prototype (from ezbee/app_signals.h):
 *   typedef bool (*ezb_app_signal_handler_t)(const ezb_app_signal_t *app_signal);
 *
 * Registered via ezb_app_signal_add_handler(). Runs in the stack mainloop task.
 * znp_uart_send_raw() is the independent TX path — safe to call here.
 *
 * LOCK DISCIPLINE — this runs INSIDE the stack callback context: the esp-zigbee
 * lib ALREADY holds the Zigbee lock here, so do NOT call esp_zigbee_lock_acquire
 * in this handler (re-entry/assert), and do NOT call the cross-task helpers
 * ezb_bdb_commission / ezb_permit_join from here. Conversely, ANY esp_zb_* / ezb_*
 * call from a non-callback task (RX task, ezb_main worker) MUST be wrapped in
 * esp_zigbee_lock_acquire/release. Callbacks never lock; cross-task callers
 * always lock.
 *
 * Return true  → signal consumed (no further handlers called).
 * Return false → pass to next registered handler.
 */
static bool s_signal_handler(const ezb_app_signal_t *signal)
{
    ezb_app_signal_type_t sig_type = ezb_app_signal_get_type(signal);
    uint8_t buf[32];
    size_t  n;

    switch (sig_type) {

    /* ── BDB first-start / reboot: device booted into its network ──────────── */
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        /* T36 / FINDINGS §12 HIGH (def 1): these signals are NOT unconditional
         * success — they carry ezb_bdb_signal_simple_params_t with a status
         * field exactly like FORMATION/STEERING. A factory-new init that fails,
         * or a REBOOT that can't restore the NVRAM network, still raises the
         * signal but with a non-SUCCESS status. The old code declared net-up and
         * emitted STATE_CHANGE_IND(0x09) regardless, telling the host a DEAD
         * network was a live coordinator. Require EZB_BDB_STATUS_SUCCESS (== 0),
         * matching the FORMATION/STEERING pattern below. */
        const ezb_bdb_signal_simple_params_t *p =
            ezb_app_signal_get_params(signal);
        if (p != NULL && p->status == EZB_BDB_STATUS_SUCCESS) {
            set_net_up();                 /* cache identity BEFORE publishing up (def 2) */
            send_state_change_ind(0x09u); /* retry-on-drop, loud-log (def 5) */
            ESP_LOGI(TAG, "signal 0x%04x: device boot/reboot success, network up, state_change_ind sent",
                     sig_type);
        } else {
            /* A failed boot/reboot must not resurrect a stale net-up. Clear it
             * defensively (def 3) so a host poll reads down, not a lie. */
            set_net_down();
            ESP_LOGW(TAG, "signal 0x%04x: device boot/reboot FAILED or no params (status=%d) — network NOT up",
                     sig_type, p ? (int)p->status : -1);
        }
        return false;   /* let other handlers see it too */
    }

    /* ── BDB formation / steering: require explicit success status ──────────── */
    case EZB_BDB_SIGNAL_FORMATION:
    case EZB_BDB_SIGNAL_STEERING: {
        const ezb_bdb_signal_simple_params_t *p =
            ezb_app_signal_get_params(signal);
        if (p != NULL && p->status == EZB_BDB_STATUS_SUCCESS) {
            set_net_up();                 /* cache identity BEFORE publishing up (def 2) */
            send_state_change_ind(0x09u); /* retry-on-drop, loud-log (def 5) */
            ESP_LOGI(TAG, "signal 0x%04x: formation/steering success, network up, state_change_ind sent",
                     sig_type);
        } else {
            /* T36 / FINDINGS §12 MED (def 3): a steering/formation FAILURE must
             * clear any prior net-up + invalidate the cached identity so the NCP
             * stops reporting DEV_ZB_COORD over a network that never formed. */
            set_net_down();
            ESP_LOGW(TAG, "signal 0x%04x: formation/steering failed or no params (status=%d) — network NOT up",
                     sig_type, p ? (int)p->status : -1);
        }
        return false;   /* let other handlers see it too */
    }

    /* ── ZDO device announce ─────────────────────────────────────────────── */
    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const ezb_zdo_signal_device_annce_params_t *p =
            ezb_app_signal_get_params(signal);
        if (!p) return false;
        /*
         * ezb_extaddr_t = ezb_eui64_s { union { uint8_t u8[8]; uint64_t u64; } }
         * .u64 is in little-endian byte order (same as MT wire order).
         */
        n = znp_build_tc_dev_ind(p->short_addr, p->device_addr.u64,
                                 p->capability, buf, sizeof(buf));
        if (n) znp_uart_send_raw(buf, n);
        ESP_LOGI(TAG, "DEVICE_ANNCE nwk=0x%04x tc_dev_ind sent", p->short_addr);
        return false;
    }

    /* ── ZDO device update (joined/rejoined via TC) ───────────────────────── */
    case EZB_ZDO_SIGNAL_DEVICE_UPDATE: {
        const ezb_zdo_signal_device_update_params_t *p =
            ezb_app_signal_get_params(signal);
        if (!p) return false;
        /* Use capability=0 — device update doesn't carry capability byte;
         * the P4 parser treats it as informational. */
        n = znp_build_tc_dev_ind(p->short_addr, p->device_addr.u64,
                                 0x00u, buf, sizeof(buf));
        if (n) znp_uart_send_raw(buf, n);
        ESP_LOGI(TAG, "DEVICE_UPDATE nwk=0x%04x tc_dev_ind sent", p->short_addr);
        return false;
    }

    /* ── ZDO leave indication (child/neighbor left) ──────────────────────── */
    case EZB_ZDO_SIGNAL_LEAVE_INDICATION: {
        const ezb_zdo_signal_leave_indication_params_t *p =
            ezb_app_signal_get_params(signal);
        if (!p) return false;
        /*
         * znp_build_leave_ind takes remove(1)+rejoin(1) bytes.
         * ezb_zdo_leave_indication_params_t carries leave_type (enum).
         * Map: LEAVE_TYPE_REJOIN → rejoin=1, remove=0; else remove=1, rejoin=0.
         * (ezb_zdo_leave_type_t values: grep shows EZB_ZDO_LEAVE_TYPE_REJOIN).
         */
        uint8_t rejoin = (p->leave_type == EZB_ZDO_LEAVE_TYPE_REJOIN) ? 1u : 0u;
        uint8_t remove = rejoin ? 0u : 1u;
        n = znp_build_leave_ind(p->short_addr, p->device_addr.u64,
                                remove, rejoin, buf, sizeof(buf));
        if (n) znp_uart_send_raw(buf, n);
        ESP_LOGI(TAG, "LEAVE_INDICATION nwk=0x%04x leave_ind sent", p->short_addr);
        return false;
    }

    /* ── ZDO self-leave: THIS coordinator left its network ───────────────── *
     * T36 / FINDINGS §12 MED (def 3). Distinct from LEAVE_INDICATION (a child
     * left): EZB_ZDO_SIGNAL_LEAVE means our own node left (e.g. a reset-type
     * leave). The network is gone — clear s_net_up + invalidate the cached
     * identity so the NCP stops reporting DEV_ZB_COORD with a stale panid. */
    case EZB_ZDO_SIGNAL_LEAVE: {
        set_net_down();
        ESP_LOGW(TAG, "ZDO_SIGNAL_LEAVE: self left network — net DOWN, cache invalidated");
        return false;
    }

    /* ── NWK failure: the network reported a failure status ──────────────── *
     * T36 / FINDINGS §12 MED (def 3). EZB_NWK_SIGNAL_NETWORK_STATUS carries an
     * ezb_nwk_network_status_t error code. We do not have a fine-grained
     * "network terminated" signal here, so on a reported NWK failure we
     * conservatively mark the network down so the host re-polls / re-commissions
     * rather than trusting a possibly-dead network as up. */
    case EZB_NWK_SIGNAL_NETWORK_STATUS: {
        const ezb_nwk_signal_network_status_params_t *p =
            ezb_app_signal_get_params(signal);
        set_net_down();
        ESP_LOGW(TAG, "NWK_SIGNAL_NETWORK_STATUS status=0x%02x — net marked DOWN",
                 p ? (unsigned)p->status : 0xFFu);
        return false;
    }

    default:
        /* Unhandled — do not consume */
        return false;
    }
}

/* ── Backend table ───────────────────────────────────────────────────────── */

const znp_backend_t *znp_ezb_backend(void)
{
    static const znp_backend_t b = {
        .get_ieee             = ezb_get_ieee,
        .request_reset        = ezb_request_reset,
        .apply_config         = ezb_apply_config,
        .start_stack          = ezb_start_stack,
        .bdb_commission       = ezb_bdb_commission,
        .get_nwk_info         = ezb_get_nwk_info,
        .permit_join          = ezb_permit_join,
        .request_factory_new  = ezb_request_factory_new,
    };
    return &b;
}
