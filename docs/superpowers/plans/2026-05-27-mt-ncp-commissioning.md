# esp-znp-core — Phase 4: Commissioning → Network Forms Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Drive the ESP32-C6/H2 NCP through host-controlled commissioning so it forms a Zigbee coordinator network and reports "network up" — i.e. `zhac-main-core` (P4) completes its `do_commissioning` + `coordinator_start` sequence against this NCP.

**Architecture:** Same split as link-up. The **dispatcher (`znp_dispatch`) stays pure + host-tested**: it buffers the host's NV-config writes into a `znp_netcfg_t`, and on `ZDO_STARTUP_FROM_APP` / `APP_CNF` BDB commands it calls **extended `znp_backend_t` ops** (`apply_config`, `start_stack`, `bdb_commission`, `get_nwk_info`, `register_endpoint`) and builds the SRSP/AREQ bytes. The **`znp_ezb` backend** implements those ops via the verified v2.x `ezb_*`/`esp_zigbee_*` API and runs an **app signal handler** that turns `EZB_BDB_SIGNAL_*` / `EZB_ZDO_SIGNAL_*` into MT AREQs (`ZDO_STATE_CHANGE_IND`, `TC_DEV_IND`, `LEAVE_IND`) sent via `znp_uart_send_raw`.

**Tech Stack:** esp-zigbee-lib v2.0.1 (`ezb_*`/`esp_zigbee_*`), `mt_proto`, `znp_dispatch`/`znp_ezb`/`znp_uart` (from link-up). Host CTest for dispatch; on-target build for glue.

**Builds on:** branch `main` (link-up complete). Deferred to later phases: ZDO interview (node/active-ep/simple desc — Phase 5), AF data path (already have `ezb_apsde_data_request` — Phase 6).

---

## A. Confirmed v2.x API (from the installed `espressif__esp-zigbee-lib/include` — use verbatim)

| Need | Symbol (header) |
|---|---|
| init | `esp_err_t esp_zigbee_init(const esp_zigbee_config_t *cfg)` — `cfg.device_config.device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR` (`esp_zigbee.h`, `ezbee/nwk.h`) |
| start (deferred) | `esp_err_t esp_zigbee_start(bool autostart)` — pass **false** (do all but skip startup) (`esp_zigbee.h`) |
| main loop | `esp_err_t esp_zigbee_launch_mainloop(void)` (`esp_zigbee.h`) |
| pan id | `void ezb_set_panid(ezb_panid_t pan_id)` (`ezbee/core.h`) |
| ext pan | `void ezb_set_use_extended_panid(const ezb_extpanid_t *extpanid)` (`ezbee/core.h`) |
| channel | `ezb_err_t ezb_set_channel_mask(uint32_t channel_mask)` (`ezbee/core.h`) |
| nwk key | `ezb_err_t ezb_secur_set_network_key(const uint8_t *key)` (`ezbee/secur.h`) |
| role | `ezb_err_t ezb_nwk_set_device_type(ezb_nwk_device_type_t)` (`ezbee/nwk.h`) |
| commission | `ezb_err_t ezb_bdb_start_top_level_commissioning(ezb_bdb_comm_mode_mask_t mode)` — `EZB_BDB_MODE_INITIALIZATION=0x01`, `NETWORK_STEERING=0x04`, `NETWORK_FORMATION=0x08` (`ezbee/bdb.h`) |
| open net | `ezb_err_t ezb_bdb_open_network(uint8_t dur)` (`ezbee/bdb.h`) |
| nwk getters | `ezb_get_panid()`, `ezb_get_current_channel()`, short addr (confirm in `ezbee/nwk.h`) |
| signals | enum `ezb_app_signal_type_e` (`ezbee/app_signals.h`): `EZB_BDB_SIGNAL_FORMATION`, `EZB_BDB_SIGNAL_STEERING`, `EZB_BDB_SIGNAL_DEVICE_FIRST_START/REBOOT`, `EZB_ZDO_SIGNAL_DEVICE_ANNCE`, `EZB_ZDO_SIGNAL_DEVICE_UPDATE`, `EZB_ZDO_SIGNAL_LEAVE_INDICATION`, `EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS` |
| endpoint reg | `ezb_af_ep_list_t` + add-ep/register fns (`ezbee/af.h`) — **implementer: confirm exact `ezb_af_*`/`esp_zigbee_*` register fn names in `ezbee/af.h` before coding 4.5** |
| signal handler | the app defines the signal handler the stack calls — **implementer: grep `ezbee/app_signals.h` + `esp_zigbee.h` for the exact prototype (`ezb_application_signal_handler` / `esp_zb_app_signal_handler`) + the params accessor; confirm before 4.5/4.6** |

## B. Wire-contract authority (incoming byte layouts)
The dispatcher must parse **exactly what the P4 sends**. The canonical layouts live in `zhac-components/components/zigbee_mgr/zigbee_mgr.cpp` — each task below cites the P4 function to mirror byte-for-byte, and host tests must use vectors captured from that layout. NV item IDs (from `zigbee_mgr.cpp`, confirmed): `STARTUP_OPTION=0x0003`, `LOGICAL_TYPE=0x0087`, `PANID=0x0083`, `EXTENDED_PAN_ID=0x002D`, `CHANLIST=0x0084`, `PRECFGKEY=0x0062`, `PRECFGKEYS_ENABLE=0x0063`.

MT commands handled this phase (all cmd0 `MT_SREQ(ZNP_SYS|ZNP_ZDO|ZNP_APP_CNF)`):
SYS NV: `NV_ITEM_INIT 0x07`, `NV_WRITE 0x09`/`WRITE_EXT 0x1D`, `NV_READ 0x08`/`READ_EXT 0x1C`, `NV_LENGTH 0x13`, `NV_DELETE 0x12`. ZDO: `STARTUP_FROM_APP 0x40`, `EXT_NWK_INFO 0x50`, `MGMT_PERMIT_JOIN 0x36`, `MSG_CB_REGISTER 0x3E`. APP_CNF: `BDB_SET_CHANNEL 0x08`, `BDB_START_COMMISSIONING 0x05`. AF: `AF_REGISTER 0x00`. AREQs emitted: `ZDO_STATE_CHANGE_IND 0x45/0xC0`, `ZDO_TC_DEV_IND 0x45/0xCA`, `ZDO_LEAVE_IND 0x45/0xC9`.

---

## Task 4.1: `znp_netcfg` + NV-write/read config buffering (host TDD)

**Files:** Modify `components/znp_dispatch/include/znp_dispatch.h`, `znp_dispatch.c`, `test/test_znp_dispatch.cpp`.

Add a config struct + handle the SYS NV commands by buffering config and acking with status 0x00. **Read `zhac-components/components/zigbee_mgr/zigbee_mgr.cpp` `nv_write_raw` (cmd1 0x1D) + `nv_read` (0x1C) to mirror the exact payload layout** (`[id:2 LE][offset:2 LE][len:1][data:len]` for WRITE_EXT — CONFIRM against the P4 source, it is the authority).

- [ ] **Step 1: Add to `znp_dispatch.h`:**
```c
/* Network config accumulated from the host's NV writes, applied at STARTUP_FROM_APP. */
typedef struct {
    uint16_t pan_id;        bool have_pan_id;
    uint8_t  ext_pan_id[8]; bool have_ext_pan_id;
    uint32_t chan_mask;     bool have_chan_mask;
    uint8_t  nwk_key[16];   bool have_nwk_key;
    uint8_t  logical_type;  bool have_logical_type;
} znp_netcfg_t;

/* NV item IDs (TI Z-Stack), as the P4 zigbee_mgr emits them. */
#define ZNP_NV_STARTUP_OPTION 0x0003
#define ZNP_NV_LOGICAL_TYPE   0x0087
#define ZNP_NV_PANID          0x0083
#define ZNP_NV_EXTENDED_PANID 0x002D
#define ZNP_NV_CHANLIST       0x0084
#define ZNP_NV_PRECFGKEY      0x0062

/* Record one NV write into cfg (id + value bytes). Returns true if recognized/stored
 * (unrecognized ids are accepted/ignored). Pure — host-testable. */
bool znp_netcfg_apply_nv(znp_netcfg_t *cfg, uint16_t id, const uint8_t *val, uint8_t len);
```
Include `<stdbool.h>` if not already.

- [ ] **Step 2 (TDD): add `test_netcfg()` to the test** asserting: a PANID write (`id=0x0083`, val `{0x34,0x12}`) sets `cfg.pan_id==0x1234` + `have_pan_id`; a CHANLIST write (`0x0084`, 4 bytes LE for channel 15 mask `0x00008000`) sets `chan_mask`; a 16-byte PRECFGKEY sets `nwk_key` + `have_nwk_key`; LOGICAL_TYPE `{0x00}` → `logical_type==0` `have_logical_type`; an unknown id returns true without corrupting cfg. Wire it into `main()`. Run → FAIL.

- [ ] **Step 3: implement `znp_netcfg_apply_nv`** in `znp_dispatch.c` (switch on id, `memcpy`/little-endian decode into the matching field + set the `have_*` flag; default: return true, ignore). Run host tests → PASS.

- [ ] **Step 4:** Extend `znp_dispatch` so SYS NV commands are handled: WRITE/WRITE_EXT parse `[id][offset][len][data]` (mirror P4 layout) → `znp_netcfg_apply_nv` on a `static znp_netcfg_t` (file-scope in dispatch, or passed via a `znp_dispatch_ctx`) → reply SRSP with status `0x00`. NV_ITEM_INIT/LENGTH/DELETE/READ → reply a benign success SRSP (READ returns the buffered value or zeros + status 0x00). Add host tests asserting the SRSP framing (status byte 0x00) for a WRITE_EXT. **Design note:** introduce a `znp_dispatch_ctx { znp_netcfg_t cfg; const znp_backend_t *be; }` passed to `znp_dispatch` (replaces the bare `be` param) so config state has a home and stays host-testable — update the link-up call site in `app_main` accordingly and re-run the existing dispatch tests (they must still pass with the ctx form).

- [ ] **Step 5: firmware build green + commit** (`feat(znp_dispatch): buffer NV config writes (commissioning step 1)`).

*(Tasks 4.2–4.6 follow the same shape; full specs below.)*

---

## Task 4.2: STARTUP_FROM_APP + BDB commissioning dispatch (host TDD)

Extend `znp_backend_t` with commissioning ops; dispatch the start/commission commands. **Mirror P4 `coordinator_start` (ZDO_STARTUP_FROM_APP 0x40) + `do_commissioning` (APP_CNF 0x08 set-channel, 0x05 start-commissioning) in `zigbee_mgr.cpp`.**

- [ ] **Step 1: extend `znp_backend_t`** (in `znp_dispatch.h`) with:
```c
    void (*apply_config)(const znp_netcfg_t *cfg);   /* push buffered cfg into the stack */
    bool (*start_stack)(void);                       /* esp_zigbee_init(coordinator)+start(false)+launch */
    bool (*bdb_commission)(uint8_t mode_mask);       /* ezb_bdb_start_top_level_commissioning */
    bool (*get_nwk_info)(uint16_t *panid, uint16_t *short_addr, uint8_t *dev_state);
    bool (*permit_join)(uint8_t duration_s);
```
(Keep `get_ieee`/`request_reset` from link-up. Update the fake backend in tests + `znp_ezb` stubs so everything still compiles.)

- [ ] **Step 2 (TDD):** add tests with a fake backend recording calls: `ZDO_STARTUP_FROM_APP` (SREQ ZDO/0x40) → dispatch calls `apply_config` then `start_stack`, replies SRSP status 0x00 (mirror P4's expected SRSP). `APP_CNF BDB_START_COMMISSIONING` (SREQ APP_CNF/0x05, payload = mode byte; **map TI mode → v2 `EZB_BDB_MODE_*`: formation 0x08, steering 0x04**) → calls `bdb_commission(mode)`. `APP_CNF BDB_SET_CHANNEL` (0x08) → records channel (or no-op + status ok). Wire in, run → FAIL.

- [ ] **Step 3: implement** the dispatch branches (ZDO 0x40, APP_CNF 0x05/0x08), translating the TI BDB mode byte to the `EZB_BDB_MODE_*` value before calling `bdb_commission`. Run → PASS. Firmware build green. Commit.

---

## Task 4.3: ZDO_EXT_NWK_INFO + AF_REGISTER dispatch (host TDD)

**Mirror P4 ext-nwk-info poll (ZDO 0x50, reads panid/shortaddr/devstate) + `AF_REGISTER` (AF 0x00) in `zigbee_mgr.cpp`.**

- [ ] **Step 1 (TDD):** `ZDO_EXT_NWK_INFO` (SREQ ZDO/0x50) → dispatch calls `be->get_nwk_info(&panid,&short,&state)` and builds the SRSP payload in the exact field order/offsets the P4 reads (CONFIRM layout from the P4 0x50 handler). `AF_REGISTER` (SREQ AF/0x00) → reply SRSP status 0x00 (the NCP registers its endpoint at start; AF_REGISTER just needs a success ack for link-up — record the endpoint for later phases). Fake backend returns known panid/short/state; assert SRSP bytes. Run → FAIL.
- [ ] **Step 2: implement** + PASS + build + commit.

---

## Task 4.4: AREQ builders — STATE_CHANGE_IND / TC_DEV_IND / LEAVE_IND (host TDD)

Pure encoders (like `znp_build_reset_ind`), host-tested with exact vectors. **Mirror the payloads the P4 AREQ handlers parse** (`on_state_change` 0xC0, `on_tc_dev_ind` 0xCA, `on_zdo_leave_ind` 0xC9 in `zigbee_mgr.cpp`).

- [ ] **Step 1 (TDD):** add `size_t znp_build_state_change_ind(uint8_t dev_state, uint8_t *buf, size_t cap)` (AREQ ZDO/0xC0, payload `{dev_state}`); `znp_build_tc_dev_ind(uint16_t nwk, uint64_t ieee, uint8_t parent_nwk?, ...)` (ZDO/0xCA — mirror P4 `on_tc_dev_ind` field order); `znp_build_leave_ind(...)` (ZDO/0xC9). Assert each against a hand-computed vector (FCS via `mt_fcs`). Run → FAIL.
- [ ] **Step 2: implement** the builders via `mt_encode` + PASS + build + commit.

---

## Task 4.5: `znp_ezb` commissioning impl + signal handler (on-target glue)

Implement the new `znp_backend_t` ops with the confirmed v2.x API, and the app signal handler that emits AREQs. **No host test — build-verified; greps the real headers for the few flagged unknowns.**

- [ ] **Step 1:** Implement in `znp_ezb.c` (confirm exact endpoint-register + signal-handler symbols in `ezbee/af.h` + `app_signals.h` first):
  - `ezb_apply_config(cfg)`: if `have_pan_id` → `ezb_set_panid`; `have_ext_pan_id` → `ezb_set_use_extended_panid`; `have_chan_mask` → `ezb_set_channel_mask`; `have_nwk_key` → `ezb_secur_set_network_key`; role → `ezb_nwk_set_device_type(EZB_NWK_DEVICE_TYPE_COORDINATOR)`.
  - `ezb_start_stack()`: build `esp_zigbee_config_t{ .device_config.device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR }`, `esp_zigbee_init(&cfg)`, `esp_zigbee_start(false)`, then `esp_zigbee_launch_mainloop()` on its own task. Return ok.
  - `ezb_bdb_commission(mode)`: `ezb_bdb_start_top_level_commissioning(mode)`.
  - `ezb_get_nwk_info(...)`: `ezb_get_panid()`, short addr getter, dev-state (coordinator → up state once formed).
  - `ezb_permit_join(dur)`: `ezb_bdb_open_network(dur)`.
- [ ] **Step 2:** Implement the **app signal handler** (the prototype confirmed from headers): on `EZB_BDB_SIGNAL_FORMATION`/`STEERING`/`DEVICE_FIRST_START`/`REBOOT` → build `STATE_CHANGE_IND` (dev_state reflecting up) via `znp_build_state_change_ind` → `znp_uart_send_raw`; on `EZB_ZDO_SIGNAL_DEVICE_ANNCE`/`DEVICE_UPDATE` → `TC_DEV_IND`; on `EZB_ZDO_SIGNAL_LEAVE_INDICATION` → `LEAVE_IND`. (The signal handler needs the UART send + builders — include `znp_uart.h` + `znp_dispatch.h`.)
- [ ] **Step 3:** Build green (this is where v2.x API mismatches surface — fix signatures against the installed headers; report BLOCKED only if a needed API genuinely doesn't exist). Commit.

---

## Task 4.6: Wire app_main + integration

- [ ] **Step 1:** Update `app_main`/`on_frame` to use the `znp_dispatch_ctx` (cfg + `znp_ezb_backend()`); ensure the signal handler is registered/linked (per the confirmed mechanism — weak symbol the app defines, or an explicit register call). Keep boot `SYS_RESET_IND`.
- [ ] **Step 2:** Build green.
- [ ] **Step 3 (hardware — SKIP here, no board):** P4 integration — flash C6, wire to P4, watch the P4 log progress through `do_commissioning` → `Network up: shortaddr=0x0000 ... panid=...`. Document as hardware-pending.
- [ ] **Step 4:** Commit (`feat: commissioning wired — coordinator forms network (P4 network-up)`).

---

## C. Self-Review
- Covers the P4 commissioning command set (NV config, STARTUP_FROM_APP, BDB set-channel + start, EXT_NWK_INFO, AF_REGISTER, permit-join) + the 3 AREQs (STATE_CHANGE/TC_DEV/LEAVE). ZDO interview + AF data are Phases 5/6 (out of scope).
- Host-testable: config buffering (4.1), command dispatch (4.2/4.3), AREQ builders (4.4). On-target/build-verified: ezb glue + signal handler (4.5/4.6).
- v2.x API embedded from the installed headers (§A); BDB mode values corrected (FORMATION=0x08, STEERING=0x04). `esp_zigbee_start(false)` gives the deferred, host-driven commissioning.
- **Wire layouts: P4 `zigbee_mgr` builders are the authority** — each task cites the function to mirror + requires host vectors captured from it. This is the one real risk: exact NV/STARTUP/EXT_NWK_INFO byte offsets must match the P4 — do NOT guess; read the source.

## D. Open items (resolve at task time)
- Exact endpoint-register + app-signal-handler symbols/prototype in v2.x (`ezbee/af.h`, `app_signals.h`) — grep before 4.5.
- Exact P4 payload byte layouts for NV_WRITE_EXT, ZDO_STARTUP_FROM_APP SRSP, ZDO_EXT_NWK_INFO SRSP, TC_DEV_IND/LEAVE_IND — read `zigbee_mgr.cpp`.
- Whether `esp_zigbee_start(false)` + an explicit later `ezb_bdb_start_top_level_commissioning` is the correct v2.x deferred-formation sequence, or formation auto-runs after start — verify against an esp-zigbee coordinator example in the installed component.
- Dev-state value the P4 expects in STATE_CHANGE_IND / EXT_NWK_INFO to consider the network "up" (TI `DEV_ZB_COORD=0x09`) — confirm from the P4 `on_state_change` / 0x50 handler.
