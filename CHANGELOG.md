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

- **lie-success NV statuses** (MED, FINDINGS §12, T33, `znp_dispatch.c:276,:47`):
  truncated/rejected NV writes returned 0x00-success. `znp_netcfg_apply_nv` now
  actually returns false on short known-id values (the header's accept/reject
  promise restored), and the write SRSP surfaces `NV_OPER_FAILED` (0x0A) instead
  of fake success.
- **NV_READ always SUCCESS+len0** (LOW, FINDINGS §12, T33, `znp_dispatch.c:110`):
  uninitialized NV reads now return `NV_ITEM_UNINIT` (0x02) for unknown ids and a
  readback from staged cfg for known ones (z2m-compat roadmap).
