# Contributing to esp-znp-core

ESP32-C6 / ESP32-H2 firmware — TI-ZNP-compatible Zigbee NCP. Speaks
the TI Z-Stack Monitor & Test (MT) protocol over UART so any
MT-driving host (e.g. ZHAC's `zhac-main-core`) can use it unchanged.

## License and CLA

Licensed under **AGPL-3.0-or-later**. All contributions require
signing the project-wide `CLA.md` (one signing covers every ZHAC
repo — see any sibling repo). Add yourself to `CONTRIBUTORS.md` in
the same pull request as your first contribution.

## Prerequisites

- **ESP-IDF v6.0** (compatible with `esp-zigbee-lib` ≥ 2.0.0). The
  `riscv32-esp-elf` toolchain is what's used for both ESP32-C6 and
  ESP32-H2.
- **`cmake` ≥ 3.16 and a host `g++`** for the host CTest suites
  (`mt_proto`, `znp_dispatch`). No board required for those.
- A real Zigbee host (e.g. `zhac-main-core` on ESP32-P4) for the
  hardware-integration smoke test once the wire link is up.

## Build

```bash
source /path/to/esp-idf-v6.0/export.sh
idf.py set-target esp32c6      # or esp32h2
idf.py build
```

## Flash + monitor

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

## Host tests (no hardware needed)

The pure logic — MT codec and the frame dispatcher — is fully
host-testable with plain CMake + CTest. Run them on every change:

```bash
(cd components/mt_proto/test \
   && cmake -B build -S . && cmake --build build && ctest --test-dir build --output-on-failure)

(cd components/znp_dispatch/test \
   && cmake -B build -S . && cmake --build build && ctest --test-dir build --output-on-failure)
```

Every wire-protocol change MUST land with a host test that asserts
the **exact byte vector on the wire** (compute FCS by hand or via
`mt_fcs`). Wire layouts mirror what the P4 host emits/parses; the
authority is
`zhac-components/components/zigbee_mgr/zigbee_mgr.cpp` — read the
relevant `nv_write_raw` / `coordinator_start` / `do_commissioning`
/ `on_*` site before coding, and mirror it byte-for-byte.

## SPDX headers for new files

```c
// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
```

## Layering rule (do not violate)

The components fan out by purpose; the host-testability depends on
this separation:

- `mt_proto` — pure MT wire codec. **No ESP-IDF includes.** Host-tested.
- `znp_dispatch` — pure request→response dispatcher + the
  `znp_backend_t` fn-pointer seam. **No `esp_zigbee.h` /
  `ezbee/*.h`** — use local `#define`s for any EZB values the
  dispatcher needs to map to. Host-tested with a fake backend.
- `znp_ezb` — chip glue. All `ezb_*` / `esp_zigbee_*` /
  `esp_read_mac` / `esp_restart` calls live here. The app signal
  handler is registered from here. On-target only.
- `znp_uart` — UART transport. Owns the TX mutex that makes
  `znp_uart_send_raw` whole-frame atomic across the RX task and the
  stack-task signal handler.
- `main/app_main.c` — wires the pieces, holds the persistent
  `znp_dispatch_ctx`, emits the boot `SYS_RESET_IND`.

If a change is creeping `esp_zigbee.h` into `znp_dispatch` or
`mt_proto`, stop — that's the layering breaking. Push the call into
`znp_ezb` and expose it via the backend seam instead.

## Adding a new ZNP MT command

1. Find the P4 builder/parser for the command in
   `zhac-components/components/zigbee_mgr/zigbee_mgr.cpp`. That
   byte layout is the spec.
2. Write a failing host test in
   `components/znp_dispatch/test/test_znp_dispatch.cpp` with the
   full encoded wire vector (FCS included).
3. Add the dispatch arm in `znp_dispatch.c` — pick the subsystem
   switch (SYS / AF / ZDO / APP_CNF) and use
   `encode_srsp_sub(subsys, cmd1, payload, …)`.
4. If a new backend op is needed, extend `znp_backend_t` (declare
   in `znp_dispatch.h`), add a NULL initializer in `znp_ezb.c`,
   and null-guard the call in dispatch. Implement the real op in
   `znp_ezb.c` (separate commit if it's a non-trivial body).
5. Run the host tests and the firmware build; both must be green.

## Commit messages

Conventional Commits: `feat(scope): subject`, `fix(scope): subject`,
`docs(scope): subject`, `chore(scope): subject`. Subject ≤ 72 chars.
Body wraps at 80. Squash review fix-ups before merge.
