# zigbee-herdsman / zigbee2mqtt compatibility — roadmap (Phase 5+)

**Status:** scoped, NOT started. Resume from this doc.
**Investigation date:** 2026-05-28.
**Target host:** zigbee-herdsman **v10.0.5** (`/home/user/webapp/zigbee/zigbee-herdsman`),
the Zigbee library that drives zigbee2mqtt. The Z-Stack adapter lives at
`src/adapter/z-stack/`; the protocol surface this firmware must match is
the TI Z-Stack 3.x **MT (Monitor & Test)** UART protocol — same wire as
the CC2652 ZNP coordinator stick.

**Current esp-znp-core state (Phase 0–4 complete, HW-validated):**
SYS PING/VERSION/GET_EXTADDR/RESET, OSAL_NV_WRITE_EXT (buffered to
`znp_netcfg_t`), OSAL_NV_READ_EXT (stub returns zeros), AF_REGISTER
(ack), ZDO_STARTUP_FROM_APP, ZDO_EXT_NWK_INFO, APP_CNF
BDB_SET_CHANNEL + BDB_START_COMMISSIONING, AREQs SYS_RESET_IND /
ZDO_STATE_CHANGE_IND / TC_DEV_IND / LEAVE_IND.

---

## 1. The blocker

zigbee-herdsman's startup picks one of three strategies based on **NV
reads**: `startup` (resume existing network), `restoreBackup` (restore
from JSON), `startCommissioning` (form a new one). It checks `NIB`
(~100-byte Network Information Base struct), `NWK_ACTIVE_KEY_INFO`,
`ZNP_HAS_CONFIGURED_ZSTACK`, plus PANID / EXTENDED_PAN_ID / CHANLIST.

**Current `osalNvReadExt` returns zeros → z2m sees "no network" on
every boot → tries to re-form, never resumes.** This is the gating
defect to fix before anything else.

## 2. MT command set zigbee-herdsman actually invokes

Extracted from `src/adapter/z-stack/adapter/*.ts` (`grep "Subsystem.X,
\"cmd\""`):

| Subsystem | Commands invoked by adapter |
|---|---|
| SYS | ping, version, getExtAddr, resetReq, **osalNvReadExt / WriteExt / Length / Delete**, **nvCreate, nvLength, nvRead, nvWrite** (legacy), **stackTune** |
| AF | register, **dataRequest** (AREQ confirm), **dataConfirm**, **dataRetrieve**, **interPanCtl** |
| ZDO | startupFromApp, **extNwkInfo**, **extRouteDisc**, **stateChangeInd** (AREQ); plus generic `requestZdo(clusterId, payload)` covering node/active-ep/simple desc, bind/unbind, mgmt-leave/permit-join/lqi/rtg, nwk-addr/ieee-addr, msg-cb-register, and the matching `*Rsp` AREQs |
| UTIL | **getDeviceInfo**, **assocAdd**, **assocGetWithAddress**, **assocRemove**, **ledControl** |
| APP_CNF | bdbSetChannel, bdbStartCommissioning, **bdbAddInstallCode** |
| SAPI | **readConfiguration**, **writeConfiguration** (legacy NV by item id) |

**Bold = not implemented today.**

## 3. NV items zigbee-herdsman reads/writes

(from `grep NvItemsIds.<X>` across the adapter)

```
NIB                ADDRMGR              APS_LINK_KEY_DATA_START   APS_LINK_KEY_TABLE
APS_USE_EXT_PANID  CHANLIST             EXTADDR                   EXTENDED_PAN_ID
EX_NWK_SEC_MATERIAL_TABLE                EX_TCLK_TABLE             LEGACY_NWK_SEC_MATERIAL_TABLE_START
LEGACY_TCLK_TABLE_START / _            LOGICAL_TYPE              NWKKEY
NWK_ACTIVE_KEY_INFO                    NWK_ALTERN_KEY_INFO       PANID
PRECFGKEY          PRECFGKEYS_ENABLE   STARTUP_OPTION            TCLK_SEED
ZCD_NV_EX_ADDRMGR                      ZCD_NV_EX_APS_KEY_DATA_TABLE
ZDO_DIRECT_CB                          ZNP_HAS_CONFIGURED_ZSTACK
```

The numeric IDs are defined in `src/adapter/z-stack/constants/common.ts`
(`NvItemsIds`). Anything we don't recognize should still ack a benign
SRSP so z2m doesn't crash; reads of unknown items return zero-length.

## 4. Data path expectation

`zStackAdapter.sendZclFrameToEndpoint` → AF `dataRequest` SREQ → SRSP
gives an MT handle → later AF `dataConfirm` AREQ with status (TI codes
`MAC_CHANNEL_ACCESS_FAILURE`, `BUFFER_FULL`, `MAC_NO_RESOURCES`,
`MAC_TRANSACTION_EXPIRED`, etc.) drives retry logic. AF `incomingMsg`
AREQ carries received ZCL frames. The retry+route-discovery loop in
`sendZclFrameToEndpointInternal` (around `adapter/zStackAdapter.ts:503`)
re-fires `dataRequest` up to 4 times and may trigger
`ZDO_EXT_ROUTE_DISC` and `requestNetworkAddress` between attempts.

## 5. Phased roadmap

Each phase = own `docs/superpowers/plans/` file, subagent-driven
execution like Phase 4, hardware-validated against a real z2m install.

| Phase | Scope | Notes |
|---|---|---|
| **5 — NV state + SYS gaps** | `osalNvReadExt` returns synthesized data: NIB built from `ezb_get_panid()`/`ezb_nwk_get_short_address()`/`ezb_get_current_channel()`/etc.; PANID / EXTENDED_PAN_ID / CHANLIST / EXTADDR via getters; `NWK_ACTIVE_KEY_INFO` from cached key; `ZNP_HAS_CONFIGURED_ZSTACK` persisted in an NVS namespace; `STARTUP_OPTION` parsed. Legacy `nvCreate`/`nvRead`/`nvWrite`/`nvLength` map to the same NV state. SAPI `readConfiguration`/`writeConfiguration` (by item id). `stackTune` = no-op SRSP. | The startup blocker. ★★★★. NIB struct layout is intricate — generate via a struct-packer helper in `znp_dispatch.c`, host-tested with the zigbee-herdsman buffalo decode as the spec. |
| **6 — AF data path** | `AF_DATA_REQUEST` (0x24/0x01) → build `ezb_apsde_data_req_t` (per ESP_ZIGBEE_SDK_V2_NOTES §8) → `ezb_apsde_data_request()`; reply SRSP with MT handle. Register `ezb_apsde_data_confirm_handler` → emit `AF_DATA_CONFIRM` AREQ (0x44/0x80) with TI-mapped status. Register `ezb_apsde_data_indication_handler` → emit `AF_INCOMING_MSG` AREQ (0x44/0x81) carrying the raw ZCL bytes. AF_INCOMING_MSG_EXT (0x82) variant if needed. | ★★★★. Status-code mapping from `ezb_apsde_data_confirm_t.status` → TI codes is the tricky bit. |
| **7 — ZDO requests + RSPs** | Dispatch arms for: NODE_DESC_REQ (0x02) → `ezb_zdo_node_desc_req`; SIMPLE_DESC_REQ (0x04) → `ezb_zdo_simple_desc_req`; ACTIVE_EP_REQ (0x05) → `ezb_zdo_active_ep_req`; NWK_ADDR_REQ (0x00) / IEEE_ADDR_REQ (0x01); BIND_REQ (0x21) / UNBIND_REQ (0x22) → `ezb_zdo_device_bind_req`/`unbind`; MGMT_LQI_REQ (0x31) / MGMT_RTG_REQ (0x32) / MGMT_LEAVE_REQ (0x34) → `ezb_zdo_device_leave_req` / MGMT_PERMIT_JOIN_REQ (0x36) → `ezb_bdb_open_network`; MSG_CB_REGISTER (0x3E). Emit each matching RSP AREQ (0x82, 0x84, 0x85, 0x80, 0x81, 0xA1, 0xA2, 0xB1, 0xB2, 0xB4, 0xB6) on the request callback. | ★★★★★ — biggest phase by command count. Many builders + matching RSP-AREQ emitters. |
| **8 — UTIL + APP_CNF + ZDO extras** | UTIL `getDeviceInfo` (0x27/0x00) — IEEE + short + dev_state + assoc list; `assocAdd/Get/Remove` — partly synthesize from stack; `ledControl` no-op. APP_CNF `bdbAddInstallCode` (`ezb_secur_ic_add`). ZDO `extRouteDisc`. | ★★. Mechanical once Phase 5/7 architecture is in place. |
| **9 — Backup/restore + edge cases** | Honour the `restoreBackup` strategy: write incoming `NIB` / key tables to the persisted NVS, re-init the stack from there on next boot. Proper TC link key handling. The `dataConfirm` retry-handler edge cases. | ★★★. Mostly about making the persisted-NV model robust. |

## 6. Hard architectural decision (deferred — record it here when made)

**NV state model:** synthesize-from-`ezb_*` vs persist-to-`nvs_flash`.

- **Synthesize** (recommended starting point): each `nvRead(itemId)`
  computes the value live from the running stack — `PANID` →
  `ezb_get_panid()`, `EXTENDED_PAN_ID` → `ezb_get_extended_panid()`,
  `CHANLIST` from current channel mask, `NIB` packed from those plus
  some constants. Simpler; the cost is that complex tables
  (ADDRMGR, APS_LINK_KEY, NWK key descriptor history) can't be
  reproduced — they're stack-internal opaque.
- **Persist**: intercept every NV write, store in our own NVS
  namespace, return on read. Matches real ZNP behavior; enables
  zigbee-herdsman backup/restore round-trip. More complex but more
  honest.
- **Recommendation:** synthesize for the read-only network state items
  (PANID/EXTPAN/CHANLIST/EXTADDR/short addr/NIB header fields) +
  persistent NVS for the few writeable flags (`HAS_CONFIGURED_ZSTACK`,
  `STARTUP_OPTION`, optionally the network key copy) — and stub the
  big opaque tables (return zero-length read). Promote to full
  persistence in Phase 9 only if z2m actually needs the round-trip.

## 7. References

- zigbee-herdsman Z-Stack adapter: `/home/user/webapp/zigbee/zigbee-herdsman/src/adapter/z-stack/`
  - `znp/definition.ts` — full MT command table with parameter layouts (the source of truth for SREQ/SRSP/AREQ payloads).
  - `znp/buffaloZnp.ts` — typed (de)serialiser for the parameter primitives.
  - `constants/common.ts` — `NvItemsIds` enum + Z-Stack version codes.
  - `adapter/zStackAdapter.ts` — high-level adapter methods, retry loops.
  - `adapter/manager.ts` — startup strategy + commissioning flow.
  - `adapter/adapter-nv-memory.ts` — NV read/write wrappers.
  - `adapter/adapter-backup.ts` — backup/restore (heavy NV).
- esp-znp-core internal docs:
  - `docs/superpowers/plans/2026-05-27-mt-ncp-foundation.md` (Phase 0–1)
  - `docs/superpowers/plans/2026-05-27-mt-ncp-link-up.md` (Phase 2–3)
  - `docs/superpowers/plans/2026-05-27-mt-ncp-commissioning.md` (Phase 4)
- ESP-Zigbee API:
  - `../../extra/docs/ESP_ZIGBEE_SDK_V2_NOTES.md` §2 (rename map), §8 (APS data primitives — directly maps to Phase 6).
  - Installed lib headers: `managed_components/espressif__esp-zigbee-lib/include/ezbee/{aps,zdo,af,nwk,secur,core}.h`.

## 8. When picking this up

1. Decide synthesize-vs-persist for Phase 5 (or accept the
   recommendation above) and record it at §6.
2. Write `docs/superpowers/plans/<YYYY-MM-DD>-mt-ncp-zh-nv-sys.md`
   following the writing-plans format (TDD per command, P4 wire
   layouts replaced by `definition.ts` + `buffaloZnp` as the spec).
3. Subagent-driven execution per phase, HW-validated against the H2
   board talking to a real `zigbee2mqtt` instance pointed at the
   resulting `/dev/ttyUSB0` (or whatever USB-UART is wired to
   UART1 GPIO5/4). z2m config snippet:
   ```yaml
   serial:
     adapter: zstack
     port: /dev/ttyUSB0
     baudrate: 115200
     rtscts: false
   ```
4. Keep `tools/test_znp.py` extended per phase — `--herdsman`
   mode that runs the same NV-read sweep zigbee-herdsman does at
   startup is the cheapest smoke between full z2m hookups.

## 9. Carry-over from current main

Carry these into Phase 5 work:
- `tools/__pycache__/` should be added to `.gitignore`.
- `znp_ezb.c:43–45` "VERIFY orientation" comment can be dropped now
  that the Espressif OUI was observed correctly via `SYS_GET_EXTADDR`.
- NV-len clamp in `znp_dispatch.c` (`dlen16 > 128 → clamp`) — switch
  to reject-with-status while in Phase 5 (we're already touching
  NV).
