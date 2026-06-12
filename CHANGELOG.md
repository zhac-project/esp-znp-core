# Changelog

All notable changes to `esp-znp-core` are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions
follow the platform-wide `vYYYYMMDDVV` scheme.

## [Unreleased]

### Changed

- **Open-source publish prep** (T38): mechanical privacy/publish scrubs ahead of
  the first public push of this repo. No live credentials or keys were found
  anywhere in the tree.
  - Untracked `tools/__pycache__/test_znp.cpython-312.pyc` (it embedded an
    absolute local build path); added `__pycache__/` and `*.py[co]` to
    `.gitignore`. The `.pyc` body held no secret, so a history rewrite was *not*
    required — it is simply untracked going forward.
  - Relocated the internal AI dev-plan artifacts under `docs/superpowers/plans/`
    out of this public repo (per the project's private-docs rule). The three
    `2026-05-27-mt-ncp-*.md` plans are preserved in the workspace `extra/` tree,
    not published. The public `docs/zigbee-herdsman-z2m-compat-roadmap.md` is
    kept.
  - Reworded dangling references to an internal `FINDINGS.md` review doc out of
    source comments and config (`sdkconfig.defaults`, `main/idf_component.yml`,
    `.github/workflows/ci.yml`, and the `znp_*` / `mt_proto` component comments):
    the `FINDINGS.md` filename, `FINDINGS §12` section tags, and bare `F##`
    shorthand are replaced with plain-English hardening descriptions. Behaviour
    is unchanged (comments/docs only). Historical `FINDINGS §12` citations in this
    changelog are left as-is as a development-history record.
  - **Pending user decision (not actioned):** the git author identity and
    `CONTRIBUTORS.md` carry a personal name + gmail. Whether to publish as-is or
    rewrite to a project/noreply identity (irreversible after first push) is the
    maintainer's call; git history and `CONTRIBUTORS.md` were left untouched.

### Security

- **UART RX recovery / MT parser robustness** (MED/LOW, FINDINGS §12, T37,
  `znp_uart.c` / `mt_proto.cpp`): hardened the UART overrun/framing-error
  recovery and the MT frame parser against garbled/noisy input so a corrupt
  byte stream resyncs cleanly without blowing the host's 2-3 s SRSP budget or
  feeding the weak XOR-8 FCS a false-accept. T35's RX-task stack/TWDT/xTaskCreate
  changes are preserved unchanged.
  - **MED — overrun recovery order** (`znp_uart.c:108`, `UART_FIFO_OVF` /
    `UART_BUFFER_FULL`): the recovery flushed the driver input *before*
    `xQueueReset(s_uart_q)`, so bytes arriving in the flush→reset window stayed
    in the ring after their event was dropped — a permanent read-accounting lag
    where every later event read stale bytes first, delaying SRSPs. Swapped the
    order: `xQueueReset()` FIRST, then `uart_flush_input()`, then
    `mt_parser_reset()` so a half-consumed frame can't poison the next parse.
  - **MED — FRAME_ERR / PARITY_ERR ignored** (`znp_uart.c`, new
    `UART_FRAME_ERR` / `UART_PARITY_ERR` case): these fell into `default` and
    were ignored, streaming wire-corrupt noise into the parser guarded only by
    the 1/256 XOR-8 FCS (a false-accepted SYS RESET 0x21/0x00 would trigger a
    spurious `esp_restart`). Now handled like an overrun — `uart_flush_input()`
    + `mt_parser_reset()` to resync on the next clean SOF.
  - **MED — accidental singleton / no deinit** (`znp_uart_init`): the driver is
    documented as a process-wide init-once singleton (the co-processor brings the
    UART up once at boot and never tears it down; a deinit would be dead teardown
    surface), now guarded by an `s_inited` flag against accidental double-init
    (which would leak the TX mutex and spawn a second RX task racing the first on
    the same event queue). The flag is cleared on the `xTaskCreate`-failure path
    so init may be retried.
  - **LOW — parser reject paths lost a valid SOF** (`mt_proto.cpp`, new
    `mt_parser_resync()` helper, LEN-reject + FCS-fail paths): both reject paths
    consumed the offending byte without re-testing it as a SOF, so an `0xFE` that
    was actually the *next* frame's start-of-frame was discarded and that valid
    frame lost. Resync is now byte-preserving — reset to SOF-hunt, then if the
    current byte is `MT_SOF` advance to the length-pending state.
  - **LOW — `mt_encode` NULL deref** (`mt_proto.cpp:12`): dereferenced
    `f->payload` with no guard when `payload_len > 0` (public-API boundary). Now
    returns 0 when `payload == NULL && payload_len > 0` (NULL+len-0 still encodes,
    e.g. SYS_PING).
  - Host tests (`mt_proto/test/test_mt_proto.cpp`): added LEN-reject-then-valid-SOF,
    FCS-fail-then-valid-SOF (valid frame NOT lost), non-SOF-reject regression
    guards, a 0xFE-flood forward-progress guard (1000 SOF bytes → one byte per
    feed, no spin / no false emit, valid frame after the flood still decodes —
    locks the one-byte-per-call invariant against a parser refactor), and
    `mt_encode(NULL, len>0)→0` cases. Full suite green.

- **Commissioning signal / network-state integrity** (HIGH/MED/LOW, FINDINGS
  §12, T36, `znp_ezb.c` / `znp_dispatch.c` / `app_main.c`): the signal handler and
  net-state tracking reported a dead network as up and left the network key in
  RAM. Fixed defensively, preserving T35's lock discipline (no lock acquire and
  no cross-task helpers added inside the signal handler).
  - **HIGH — net-up declared on a dead network** (`znp_ezb.c` `s_signal_handler`,
    `EZB_BDB_SIGNAL_DEVICE_FIRST_START` / `..._DEVICE_REBOOT`): these were treated
    as unconditional success, but they carry `ezb_bdb_signal_simple_params_t` with
    a `status` field exactly like FORMATION/STEERING. A failed factory-new init or
    a REBOOT that could not restore the NVRAM network still declared net-up and
    emitted `STATE_CHANGE_IND(0x09)` to the host — a live coordinator over a dead
    radio. Now require `EZB_BDB_STATUS_SUCCESS` (== 0), matching the
    FORMATION/STEERING case; a non-success status marks the network down instead.
  - **MED — `s_net_up` published before the identity cache** (`set_net_up`): the
    flag was set true before `cache_nwk_info()`, so a concurrent
    `ZDO_EXT_NWK_INFO` (0x50) host poll could read `dev_state=0x09` paired with a
    stale `panid 0xFFFF`. Bring-up is now one ordered op — cache identity FIRST,
    then publish `s_net_up`. The down transition (`set_net_down`) clears
    `s_net_up` first, then invalidates the cache, so the reader never sees up+wiped.
  - **MED — `s_net_up` never cleared on network loss** (`set_net_down`): the flag
    was never reset, so the NCP reported `DEV_ZB_COORD` forever after the network
    died. Self-leave (`EZB_ZDO_SIGNAL_LEAVE`) + formation/steering-fail clear
    net-up (and invalidate the cached identity); a returned `launch_mainloop` also
    clears it. Per-frame NWK status (`EZB_NWK_SIGNAL_NETWORK_STATUS`) is logged
    only — its `status` is a per-frame routing diagnostic (NO_ROUTE_AVAILABLE,
    INDIRECT_TRANSACTION_EXPIRY, …) about other devices that fires routinely on a
    healthy coordinator, so it must NOT flap `s_net_up`.
  - **MED — post-start NV config silently lost** (`ezb_apply_config`): once the
    stack is started the buffered config has already been consumed and
    `start_stack` is idempotent-true, so a post-start `apply_config` re-staged
    config that never applied — the host's NV writes were ACK'd but ignored with
    no path to take effect. Now detected: an honest loud `ESP_LOGW` states the
    config will not take effect until a reset (the host's STARTUP_OPTION-clear +
    `SYS_RESET` factory-new flow re-keys/re-PANs a running coordinator), and the
    already-consumed buffer is not overwritten.
  - **MED — network key not zeroized** (`ezb_apply_config`/bring-up +
    `znp_dispatch.c` 0x40): the cleartext network key lingered in `s_pending_cfg`
    (backend) and in the dispatch ctx (`s_ctx.cfg`, app_main) for the process
    lifetime after `ezb_secur_set_network_key` consumed it. Now `memset` to zero
    + `have_nwk_key=false` at BOTH sites once consumed (open-source release: keys
    must not persist in RAM).
  - **LOW — ignored results / silent fallbacks** (def 5): the state-bearing
    `STATE_CHANGE_IND` AREQ send result was ignored — a 200 ms TX-mutex-timeout
    silently dropped state=9 and stalled the host's 10 s commissioning gate with
    no retry; now sent via `send_state_change_ind` with bounded retry + a loud
    `ESP_LOGE` on exhaustion. `esp_read_mac` failure (serving an all-zero IEEE),
    `esp_zigbee_launch_mainloop`'s discarded return (silent task self-delete), the
    `app_main` `on_frame` SRSP send failure, and the boot `RESET_IND` build/send
    result are now all logged instead of ignored.

- **Unlocked cross-task stack calls + bring-up on the RX task** (CRIT, FINDINGS
  §12, T35, `znp_ezb.c` / `znp_uart.c`): the Zigbee stack task/lock structure was
  unsafe in four ways.
  - **CRIT — unlocked BDB commissioning** (`znp_ezb.c` `ezb_bdb_commission`):
    `ezb_bdb_start_top_level_commissioning` was called from the UART RX task
    while the esp-zigbee mainloop task ran, with NO lock held — a data race in
    the stack's BDB scheduler. The lib header (`esp_zigbee.h:175`) makes holding
    the Zigbee lock MANDATORY for any SDK API invoked outside a stack callback.
    Now wrapped in `esp_zigbee_lock_acquire(portMAX_DELAY)` /
    `esp_zigbee_lock_release` (F32/`ae4adef` had only fixed the READ path via a
    mainloop-cached identity; the WRITE entry points stayed unlocked).
  - **HIGH — unlocked `open_network`** (`ezb_permit_join` →
    `ezb_bdb_open_network`): same cross-task class, now LIVE after T34 wired the
    0x36 permit-join handler. Same lock fix. The bring-up config/AF setters
    (`ezb_set_*`, `ezb_secur_set_network_key`, `ezb_af_*`) and `esp_zigbee_start`
    are likewise lock-wrapped defensively.
  - **HIGH — stack bring-up on the RX task** (`ezb_start_stack` →
    `ezb_worker_task`): the entire ZBOSS bring-up (NVRAM erase, `esp_zigbee_init`,
    AF registration, `esp_zigbee_start`) ran from the frame callback on the 4 KB
    TWDT-subscribed RX task (5 s panic). A slow flash op could reboot the chip
    mid-commissioning, and long handlers stalled MT-frame parsing + SRSP
    timeliness. Bring-up now runs on a dedicated worker task (`ezb_main`), which
    is NOT TWDT-subscribed (it blocks forever in `esp_zigbee_launch_mainloop`).
    `ezb_start_stack` spawns the worker and returns immediately, so the 0x40
    `ZDO_STARTUP_FROM_APP` SRSP is an immediate-accept; the host
    (`zhac-components` `zigbee_mgr.cpp` `coordinator_start`) waits on the SRSP
    only as an ack, then polls the async `ZDO_STATE_CHANGE_IND` state=9 emitted
    later by the signal handler — so async bring-up is tolerated. The RX task
    stack is raised 4096 → 8192 B and its `xTaskCreate` return is now checked
    (an ignored failure left a silently RX-dead NCP). The T33 pre-init
    `zb_storage` erase still runs unconditionally BEFORE `esp_zigbee_init` in the
    new worker location.
  - **HIGH — partial-init double-init** (`ezb_start_stack`): a failure after
    `esp_zigbee_init` left the single `s_stack_inited` flag false, so a host
    STARTUP_FROM_APP retry re-ran `esp_zigbee_init` on an initialised stack and
    re-registered the signal handler (double-init UB + duplicate AREQ per
    signal). Replaced with per-step latches (`s_zb_inited`, `s_handler_added`,
    `s_af_registered`, `s_zb_started`) so a retry RESUMES from the failed step
    instead of restarting bring-up.
  - **Follow-up (cross-repo / HW soak, not a defect)** — the worker bring-up
    latency budget is now bounded by the host's wait window. The host's
    already-configured commissioning path waits for `ZDO_STATE_CHANGE_IND`
    state=9 only ~10 s (`zhac-components` `zigbee_mgr.cpp:931`), whereas the
    fresh-commission path waits 20 s. Async worker bring-up must land state=9
    inside that ~10 s window or the host times out and retries; flagged for a
    cross-repo / hardware-soak review rather than treated as a defect here.

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
