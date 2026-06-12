# Changelog

All notable changes to `esp-znp-core` are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions
follow the platform-wide `vYYYYMMDDVV` scheme.

## [Unreleased]

### Security

- **factory-reset / NV STARTUP_OPTION** (CRIT, FINDINGS §12, T33,
  `znp_dispatch.c:71` / `znp_ezb.c`): the NV `STARTUP_OPTION` (0x0003) write was
  swallowed. The host writes 0x03 (`CLEAR_STATE|CLEAR_CONFIG`) then `SYS_RESET`
  expecting a blank coordinator, but `esp_restart()` preserves the `zb_storage`
  NVRAM so the old PAN/network-key resurrected and the radio could never be
  factory-reset. Now: a clear bit latches a persistent factory-new flag in the
  default `nvs` partition (`ezb_request_factory_new`); on the NEXT boot, BEFORE
  `esp_zigbee_init`, `ezb_start_stack` erases the `zb_storage` partition and
  re-inits it. The erase point is marked "MUST run before `esp_zigbee_init`" for
  the T35 relocation. (T34: the erase and re-init are now fail-safe — on either
  failure the latch is re-armed and the chip restarts to retry the wipe rather
  than starting the stack over stale/unmounted state.)
- **NV length-guard `uint8` wrap** (HIGH, FINDINGS §12, T33,
  `znp_dispatch.c:288,:272`): `dlen=pl[4]` in `[251..255]` made
  `(uint8_t)(5+dlen)` wrap to `<=4`, passing the guard while `apply_nv`
  over-read up to 255 B of stale parser-buffer into `nwk_key`/PAN/cfg with a
  SUCCESS SRSP. Now `size_t` arithmetic on both the 0x09 and 0x1D paths.

### Fixed

- **MGMT_PERMIT_JOIN dropped — pairing dead** (HIGH, FINDINGS §12, T34,
  `znp_dispatch.c` `dispatch_zdo` case 0x36): ZDO `MGMT_PERMIT_JOIN_REQ` (0x36)
  fell into `default:return 0`, so the host (`zigbee_mgr` `zcl_commands.cpp:58`
  `zigbee_permit_join`) saw NO SRSP and burned 3× its 2000 ms `znp_sreq_retry`
  timeout per open-network attempt — pairing could never start even though a
  backend `permit_join` (`ezb_permit_join` → `esp_zb_bdb_open_network`) existed.
  Now: parse the host's exact 5-byte payload `[AddrMode 1][DstAddr 2 LE]
  [Duration 1][TCSignificance 1]` (`zcl_commands.cpp:62-71`), call
  `backend->permit_join(payload[3])`, and answer the 1-byte SRSP the host blocks
  on. Added an optional `znp_build_permit_join_rsp` (AREQ 0x45/0xB6,
  `srcaddr 2 LE + status 1`) the backend may also emit for faithful TI parity;
  the current host registers no 0xB6 handler so it is defensive only.
- **unknown-SREQ silence → triple-timeout dead-air** (HIGH, FINDINGS §12, T34,
  `znp_dispatch.c` `znp_dispatch` default fall-through): an unhandled SREQ
  returned 0 = total silence; the host actively sends AF `AF_DATA_REQUEST`
  (0x01, ×13 call sites), the ZDO interview reqs (0x02/0x04/0x05), `MGMT_LEAVE`
  (0x34) and `MSG_CB_REGISTER` (0x3E) — each burned 2-3 s × 3 retries of dead
  air. Now an unrouted SREQ answers the MT RPC-error frame (`cmd0=0x60` SRSP of
  RPC-subsystem 0, `cmd1=0x00`, payload `{errcode 0x02 = MT_RPC_ERR_COMMAND_ID,
  offending cmd0, offending cmd1}`), turning the triple-timeout into one
  immediate honest error until each command is actually implemented. This is the
  DEFAULT fall-through only — it never shadows a real handler (incl. the new
  0x36); unrouted **AREQs** stay silent (fire-and-forget). Note: this is
  protocol-correctness for generic/herdsman-class hosts. The current host
  (`znp_driver`) correlates SRSPs by subsystem+cmd1, so it does not yet match a
  `0x60/0x00` RPC-error and still times out on unknown SREQs; shortening the
  current-host timeout needs a cross-repo follow-up — teach `znp_driver` to
  recognize the MT RPC-error frame and fail the pending SREQ. The same caveat
  applies to the STARTUP `0x02` status above (the current host ignores the status
  byte).
- **AREQ-typed SYS_RESET_REQ unrouted** (MED, FINDINGS §12, T34,
  `znp_dispatch.c` `znp_dispatch`): routing matched SREQ type-bits only, so
  generic Z-Stack hosts / zigbee-herdsman that send `SYS_RESET_REQ` as an AREQ
  (`0x41/0x00`, not the SREQ form the P4 uses at `zigbee_mgr.cpp:307`) were
  ignored and the chip never reset. The AREQ form now reaches the SAME
  `request_reset` path T33 wired; no SRSP either way (host awaits `RESET_IND`).
- **STARTUP_FROM_APP failure status inverted** (MED, FINDINGS §12, T34,
  `znp_dispatch.c` `dispatch_zdo` case 0x40): a failed `start_stack` returned
  0x01, but TI semantics are 0=RESTORED / 1=NEW_NETWORK (both success) /
  2=NOT_STARTED — so a generic host (herdsman) read 0x01 as success and ran over
  a dead stack. Now returns 0x02 (NOT_STARTED) on failure, 0x00 on success.
- **PING caps over-advertised** (LOW, FINDINGS §12, T34, `znp_dispatch.h:20`):
  `ZNP_PING_CAPS 0x0179` advertised SAPI + UTIL with no dispatch route, so a
  capability-probing host (herdsman issues `UTIL_GET_DEVICE_INFO` at startup)
  trusted dead subsystems. Trimmed to only the routed ones
  (SYS|AF|ZDO|APP = 0x0119) — trimming chosen over stubbing as the honest,
  lower-risk option for now.
- **dispatch dedup/cleanup** (LOW, FINDINGS §12, T34, `znp_dispatch.c`):
  collapsed the hardwired `encode_srsp` into `encode_srsp_sub(ZNP_SYS,…)` and
  added a `put_le64` helper for the IEEE EUI64 serialization duplicated in
  `tc_dev_ind` + `leave_ind`.
- **lie-success NV statuses** (MED, FINDINGS §12, T33, `znp_dispatch.c:276,:47`):
  truncated/rejected NV writes returned 0x00-success. `znp_netcfg_apply_nv` now
  actually returns false on short known-id values (the header's accept/reject
  promise restored), and the write SRSP surfaces `NV_OPER_FAILED` (0x0A) instead
  of fake success.
- **NV_READ always SUCCESS+len0** (LOW, FINDINGS §12, T33, `znp_dispatch.c:110`):
  uninitialized NV reads now return `NV_ITEM_UNINIT` (0x02) for unknown ids and a
  readback from staged cfg for known ones (z2m-compat roadmap).
