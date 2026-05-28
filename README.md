# esp-znp-core

ESP32-C6 / ESP32-H2 firmware that speaks the **TI Z-Stack Monitor &
Test (MT) protocol** over UART. Drop-in for any host that already
drives a CC2652-ZNP stick (e.g. [ZHAC]'s `zhac-main-core`) — the host
sees an MT NCP and does not care what silicon is on the other end.
The full Zigbee stack (`esp-zigbee-lib` v2.x) runs **on the C6/H2**;
the host stays unchanged.

[ZHAC]: https://github.com/zhac-project/zhac-platform

## Status

| Phase | What                                                              | State        |
|-------|-------------------------------------------------------------------|--------------|
| 0     | IDF skeleton, `esp-zigbee-lib` v2.x linked, boots                 | done         |
| 1     | MT wire codec (FCS / encode / decode / streaming parser)          | done         |
| 2     | UART transport, RX → parser → frame callback, TX mutex            | done         |
| 3     | SYS dispatch (PING / VERSION / GET_EXTADDR / RESET) + boot RESET_IND | done      |
| 4     | Commissioning — NV buffer, `STARTUP_FROM_APP`, BDB, `EXT_NWK_INFO`, `AF_REGISTER`, AREQs | done |
| 5     | Device interview — ZDO node / active-ep / simple desc             | todo         |
| 6     | Data path — `AF_DATA_REQUEST` / APSDE indication                  | todo         |
| 7+    | Bind / unbind / permit-join refinements, leave req                | todo         |

All host-testable logic has CTest coverage (`mt_proto` + `znp_dispatch`).
End-to-end hardware integration with a real MT host is pending a
board.

## Wiring

`esp-znp-core` talks to its host over **one UART at 115200 8N1, no
flow control**. Byte-compatible with TI-ZNP MT, so any host that
already drives a CC2652-ZNP / similar can talk to this firmware
unchanged.

### Cross-wire diagram

```
 ┌──────────────────────┐                   ┌──────────────────────┐
 │     HOST MCU         │      UART         │   esp-znp-core       │
 │  (e.g. ESP32-P4)     │  115200 8N1 noFC  │   (ESP32-C6 / H2)    │
 │                      │                   │                      │
 │      TX  ────────────┼───────────────────┼──►  RX  (GPIO4 *)    │
 │      RX  ◄───────────┼───────────────────┼───  TX  (GPIO5 *)    │
 │   NRESET ────────────┼───────────────────┼──►  EN / CHIP_PU     │
 │      GND ────────────┼───────────────────┼───  GND              │
 └──────────────────────┘                   └──────────────────────┘

 * defaults — change in menuconfig under "ZNP UART (MT NCP link)"
```

### esp-znp-core GPIO defaults

| Signal | Pin     | Kconfig key                       | Note                       |
|--------|--------:|-----------------------------------|----------------------------|
| TX     | `GPIO5` | `CONFIG_ZNP_UART_TX_GPIO`         | → host RX                  |
| RX     | `GPIO4` | `CONFIG_ZNP_UART_RX_GPIO`         | ← host TX                  |
| EN     | board strap | (`EN` / `CHIP_PU` pin)        | ← host NRESET (optional)   |
| —      | —       | `CONFIG_ZNP_UART_PORT` = `1`      | UART1                      |
| —      | —       | `CONFIG_ZNP_UART_BAUD` = `115200` | must match host            |

Override via `idf.py menuconfig → Component config → ZNP UART (MT NCP link)`.

### Reset handshake

On **every boot** the firmware emits `SYS_RESET_IND` (AREQ
`0x41/0x80`) so the host knows the NCP came back. Two ways to reset:

- **Hard reset:** host toggles the wired `EN`/`CHIP_PU` line low →
  high; the chip reboots and emits `SYS_RESET_IND`.
- **Soft reset:** host sends `SYS_RESET_REQ` (SREQ `0x21/0x00`);
  the firmware calls `esp_restart()`, reboots, and emits
  `SYS_RESET_IND` on the next boot. No SRSP is sent — the host
  waits for the AREQ.

Both paths produce the same indication on the wire. Hard reset is
recommended for production recovery from a hung stack; soft reset is
enough for bring-up and is what existing ZHAC hosts use.

### Host-side reference (ZHAC ESP32-P4)

ZHAC's `zhac-main-core` is the reference host. Its `znp_driver`
defaults (in `znp_transport.cpp`) are TX = `GPIO16`, RX = `GPIO17`,
NRESET = `GPIO28` — but every pin is overridable via
`CONFIG_ZHAC_ZNP_UART_TX_GPIO` / `_RX_GPIO` / `_RST_GPIO`. Match the
two sides up however your board is wired.

### Reference: Espressif S3-H2 board

Espressif's ESP-Thread-Border-Router / Zigbee-Gateway board carries
an ESP32-S3 host and an ESP32-H2 radio on one PCB. The hard-wired
trace between the two is:

```
H2  TXD0  ─────►  S3  GPIO17    (= S3-side host RX)
H2  RXD0  ◄─────  S3  GPIO18    (= S3-side host TX)
```

`TXD0`/`RXD0` are the H2's UART0 pins (default IOMUX `GPIO24` /
`GPIO23` — verify against the board schematic). On this board, to
use the link for the MT NCP, configure esp-znp-core on the H2 side:

```
CONFIG_ZNP_UART_PORT     = 0
CONFIG_ZNP_UART_TX_GPIO  = 24      # H2 U0TXD (verify schematic)
CONFIG_ZNP_UART_RX_GPIO  = 23      # H2 U0RXD (verify schematic)
```

UART0 is normally the ESP-IDF console — redirect it onto the H2's
USB-Serial-JTAG so logs don't collide with MT frames. In
`idf.py menuconfig`:

```
Component config → ESP System Settings → Channel for console output
    → "USB Serial/JTAG Controller"
```

The S3-side MT host (your own MT-driving firmware on the S3 of this
board) pairs symmetrically:

```
HOST_TX = GPIO18    (→ H2 RXD0)
HOST_RX = GPIO17    (← H2 TXD0)
```

The S3 board exposes its USB for flashing/monitor on a separate
USB-Serial controller, so its console is unaffected.

### C6 vs H2 — which chip to pick

Both work — the firmware is target-agnostic above the IDF layer.

- **ESP32-H2** — purpose-built 802.15.4 + BLE radio, no WiFi, lower
  cost. The natural choice for a dedicated Zigbee NCP.
- **ESP32-C6** — 802.15.4 + BLE + WiFi, stronger CPU. Pick this only
  if a future build needs WiFi or BLE on the NCP itself; otherwise
  its WiFi radio is wasted as an NCP.

Switch targets with `idf.py set-target esp32c6` or `idf.py set-target
esp32h2`. No code change.

## Build & flash

```bash
git clone https://github.com/zhac-project/esp-znp-core.git
cd esp-znp-core
source /path/to/esp-idf-v6.0/export.sh

idf.py set-target esp32c6              # or esp32h2
idf.py build

idf.py -p /dev/ttyUSB0 flash monitor
```

## Host tests (no hardware)

Two CTest suites cover the pure logic — wire codec and dispatcher —
and run anywhere a host compiler is available:

```bash
(cd components/mt_proto/test \
   && cmake -B build -S . && cmake --build build && ctest --test-dir build --output-on-failure)

(cd components/znp_dispatch/test \
   && cmake -B build -S . && cmake --build build && ctest --test-dir build --output-on-failure)
```

Both must be 100 % before any change merges. Wire-byte vectors and
FCS are asserted exactly — the codec and dispatcher are guaranteed
byte-identical to the P4 host they target.

## Wire protocol

TI Z-Stack Monitor & Test (MT) framing:

```
SOF (0xFE) | LEN (1B) | CMD0 (1B) | CMD1 (1B) | DATA (LEN bytes) | FCS (1B)
                                                                  ^
                                                            FCS = LEN ^ CMD0 ^ CMD1 ^ DATA bytes
```

Max payload 250 bytes. CMD0 high nibble selects the type:
`0x20` SREQ, `0x40` AREQ, `0x60` SRSP. Low nibble is the subsystem:
`0x01` SYS, `0x04` AF, `0x05` ZDO, `0x0F` APP_CNF.

### Implemented commands (Phase 0 – 4)

| Dir  | CMD0 / CMD1 | Name                              |
|------|-------------|-----------------------------------|
| SREQ | `0x21/0x00` | `SYS_RESET_REQ`                   |
| SREQ | `0x21/0x01` | `SYS_PING`                        |
| SREQ | `0x21/0x02` | `SYS_VERSION`                     |
| SREQ | `0x21/0x04` | `SYS_GET_EXTADDR`                 |
| SREQ | `0x21/0x07…0x1D` | `SYS_OSAL_NV_*` (buffered into config) |
| SREQ | `0x24/0x00` | `AF_REGISTER`                     |
| SREQ | `0x25/0x40` | `ZDO_STARTUP_FROM_APP`            |
| SREQ | `0x25/0x50` | `ZDO_EXT_NWK_INFO`                |
| SREQ | `0x2F/0x05` | `APP_CNF_BDB_START_COMMISSIONING` |
| SREQ | `0x2F/0x08` | `APP_CNF_BDB_SET_CHANNEL`         |
| AREQ | `0x41/0x80` | `SYS_RESET_IND`                   |
| AREQ | `0x45/0xC0` | `ZDO_STATE_CHANGE_IND`            |
| AREQ | `0x45/0xC9` | `ZDO_LEAVE_IND`                   |
| AREQ | `0x45/0xCA` | `ZDO_TC_DEV_IND`                  |

Per-phase implementation notes are in `docs/superpowers/plans/`.

## Tree

```
esp-znp-core/
├── main/                            (firmware entry, NVS init, UART wire-up,
│                                     boot RESET_IND emission)
├── components/
│   ├── mt_proto/                    (MT wire codec — FCS, encode, decode,
│   │                                 streaming parser; host CTest)
│   ├── znp_uart/                    (UART transport — 115200 8N1 noFC, RX
│   │                                 task → parser → frame callback,
│   │                                 frame-atomic TX mutex)
│   ├── znp_dispatch/                (Pure dispatcher — host CTest; SYS / ZDO
│   │                                 / APP_CNF / AF; NV config buffer;
│   │                                 AREQ builders; znp_backend_t fn-ptr seam)
│   └── znp_ezb/                     (Backend impl — esp_read_mac, esp_restart,
│                                     ezb_* commissioning ops, app signal
│                                     handler → MT AREQs)
├── docs/superpowers/plans/          (per-phase implementation plans)
├── CMakeLists.txt
├── sdkconfig.defaults               (target esp32c6, ZB_ENABLED, native radio,
│                                     custom partition table)
└── partitions.csv                   (zb_storage = NVS per esp-zigbee-sdk v2.x;
                                      zb_fct = readonly factory blob)
```

## License

Licensed under **AGPL-3.0-or-later**. See `LICENSE` for the full
notice and the additional contributor terms. Third-party components
— notably `esp-zigbee-lib`, pulled as a managed component from the
Espressif Component Registry — retain their own upstream licenses.

## Contributors

See `CONTRIBUTORS.md`. New contributors must sign the project-wide
`CLA.md` (in any ZHAC repo; see `CONTRIBUTING.md` for the process).
