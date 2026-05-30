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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* esp-zigbee-lib v2.0.1 native API */
#include "esp_zigbee.h"             /* esp_zigbee_init/start/launch_mainloop, esp_zigbee_config_t */
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

static void cache_nwk_info(void)
{
    s_cached_panid = (uint16_t)ezb_nwk_get_panid();
    s_cached_short = (uint16_t)ezb_nwk_get_short_address();
}

/* ── IEEE / reset (unchanged from stub) ─────────────────────────────────── */

static void ezb_get_ieee(uint8_t out[8])
{
    uint8_t mac[8] = {0};
    /* 802.15.4 EUI64 from efuse. esp_read_mac returns it MSB-first; the MT
     * ExtAddr field is little-endian, so reverse. VERIFY orientation on first
     * P4 hardware bring-up (compare P4 log vs chip label). */
    if (esp_read_mac(mac, ESP_MAC_IEEE802154) == ESP_OK) {
        for (int i = 0; i < 8; i++) out[i] = mac[7 - i];
    } else {
        memset(out, 0, 8);
    }
}

static void ezb_request_reset(void)
{
    esp_restart();   /* reboots → app_main emits SYS_RESET_IND on next boot */
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
static bool         s_stack_inited     = false;

static void ezb_apply_config(const znp_netcfg_t *cfg)
{
    if (!cfg) return;
    s_pending_cfg      = *cfg;
    s_have_pending_cfg = true;
}

/* ── Signal handler — forward-declared for use in ezb_start_stack ───────── */
static bool s_signal_handler(const ezb_app_signal_t *signal);

/* ── Mainloop trampoline task ────────────────────────────────────────────── */
static void mainloop_task(void *arg)
{
    (void)arg;
    /* Blocks until stack shuts down (never in normal operation) */
    esp_zigbee_launch_mainloop();
    vTaskDelete(NULL);
}

/* ── 2. start_stack ──────────────────────────────────────────────────────── */

static bool ezb_start_stack(void)
{
    if (s_stack_inited) return true;   /* idempotent — host may resend STARTUP_FROM_APP */

    /* 1. Stack init (must run BEFORE any ezb_set_*) */
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

    /* 2. Signal handler — register before anything that could emit a signal */
    ezb_err_t herr = ezb_app_signal_add_handler(s_signal_handler);
    if (herr != 0) {
        ESP_LOGE(TAG, "ezb_app_signal_add_handler failed: %d", herr);
        return false;
    }

    /* 3. Apply the buffered network parameters AFTER init */
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
        ESP_LOGE(TAG, "AF descriptor alloc failed");
        return false;
    }
    ezb_af_device_add_endpoint_desc(dev, ep_desc);  /* dev owns ep_desc after this */
    ezb_err_t reg_err = ezb_af_device_desc_register(dev);
    if (reg_err != 0) {
        ezb_af_free_device_desc(dev);
        ESP_LOGE(TAG, "ezb_af_device_desc_register failed: %d", reg_err);
        return false;
    }

    /* 5. autostart=false — host drives BDB via BDB_START_COMMISSIONING */
    err = esp_zigbee_start(false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_zigbee_start failed: %d", err);
        return false;
    }

    /* 6. Run the stack mainloop in a dedicated task */
    BaseType_t rc = xTaskCreatePinnedToCore(
        mainloop_task, "ezb_main", 8192, NULL, 5, NULL, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        return false;
    }

    s_stack_inited = true;
    return true;
}

/* ── 3. bdb_commission ───────────────────────────────────────────────────── */

static bool ezb_bdb_commission(uint8_t ezb_mode)
{
    ezb_err_t err = ezb_bdb_start_top_level_commissioning(
        (ezb_bdb_comm_mode_mask_t)ezb_mode);
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
    ezb_err_t err = ezb_bdb_open_network(dur);
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
        /* These signals mean the stack has initialised/rejoined its network.
         * No status check required — the signal itself is the success indicator. */
        s_net_up = true;
        cache_nwk_info();
        n = znp_build_state_change_ind(0x09u, buf, sizeof(buf));
        if (n) znp_uart_send_raw(buf, n);
        ESP_LOGI(TAG, "signal 0x%04x: device boot/reboot, network up, state_change_ind sent",
                 sig_type);
        return false;   /* let other handlers see it too */
    }

    /* ── BDB formation / steering: require explicit success status ──────────── */
    case EZB_BDB_SIGNAL_FORMATION:
    case EZB_BDB_SIGNAL_STEERING: {
        const ezb_bdb_signal_simple_params_t *p =
            ezb_app_signal_get_params(signal);
        if (p != NULL && p->status == EZB_BDB_STATUS_SUCCESS) {
            s_net_up = true;
            cache_nwk_info();
            n = znp_build_state_change_ind(0x09u, buf, sizeof(buf));
            if (n) znp_uart_send_raw(buf, n);
            ESP_LOGI(TAG, "signal 0x%04x: formation/steering success, network up, state_change_ind sent",
                     sig_type);
        } else {
            ESP_LOGW(TAG, "signal 0x%04x: formation/steering failed or no params (status=%d)",
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

    default:
        /* Unhandled — do not consume */
        return false;
    }
}

/* ── Backend table ───────────────────────────────────────────────────────── */

const znp_backend_t *znp_ezb_backend(void)
{
    static const znp_backend_t b = {
        .get_ieee       = ezb_get_ieee,
        .request_reset  = ezb_request_reset,
        .apply_config   = ezb_apply_config,
        .start_stack    = ezb_start_stack,
        .bdb_commission = ezb_bdb_commission,
        .get_nwk_info   = ezb_get_nwk_info,
        .permit_join    = ezb_permit_join,
    };
    return &b;
}
