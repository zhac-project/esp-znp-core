# esp-znp-core — Phase 2+3: UART Transport + SYS Dispatch (Link-Up) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ESP32-C6/H2 answer the TI-MT serial protocol over UART for the SYS subset, so `zhac-main-core` (P4) detects it as a live ZNP NCP — the first hardware-observable milestone.

**Architecture:** A UART RX task feeds bytes into the existing host-tested `mt_proto` parser; completed frames go to a **pure, host-tested dispatcher** (`znp_dispatch`) that builds byte-exact MT responses through a small `znp_backend_t` function-pointer interface. The backend impl (`znp_ezb`) is thin on-target glue (`esp_read_mac`, `esp_restart`). `znp_uart` is pure byte I/O (no encoding — the dispatcher already produces encoded bytes, keeping the wire logic testable on the host). On boot the firmware emits `SYS_RESET_IND`.

**Tech Stack:** ESP-IDF v6.0 (`driver/uart.h`, `esp_mac.h`, `esp_system.h`, FreeRTOS task), `mt_proto` (from the foundation plan), C for glue, C++ for host tests, CMake/CTest.

**Builds on:** branch `main` (foundation: `mt_proto` codec + IDF skeleton). The link-up SYS subset deliberately avoids `ezb_*`/`esp_zigbee_*` calls — those (commissioning/ZDO/APS) are Phases 4–6 (separate plans). Contract refs: `../../../../extra/docs/ZNP_API_SURFACE_C6_PORT.md` (§3 SYS rows, §5 AREQs), `../../../../extra/docs/ESP_ZIGBEE_SDK_V2_NOTES.md`.

---

## A. File Structure (this plan adds)

```
esp-znp-core/
├── main/
│   ├── CMakeLists.txt          # MODIFY: REQUIRES znp_uart, znp_dispatch, znp_ezb
│   └── app_main.c              # MODIFY: wire UART→dispatch, emit boot RESET_IND
└── components/
    ├── mt_proto/               # (exists — unchanged)
    ├── znp_dispatch/           # NEW — pure, host-tested
    │   ├── include/znp_dispatch.h
    │   ├── znp_dispatch.c
    │   ├── CMakeLists.txt
    │   └── test/{CMakeLists.txt, test_znp_dispatch.cpp}
    ├── znp_ezb/                # NEW — on-target backend impl
    │   ├── include/znp_ezb.h
    │   ├── znp_ezb.c
    │   └── CMakeLists.txt
    └── znp_uart/               # NEW — on-target UART transport
        ├── include/znp_uart.h
        ├── znp_uart.c
        ├── Kconfig
        └── CMakeLists.txt
```

**Layering (one responsibility each):** `mt_proto` (codec) ← `znp_dispatch` (req→encoded resp, pure) ← `app_main` (wiring) → `znp_uart` (bytes) + `znp_ezb` (chip glue). `znp_dispatch` depends only on `mt_proto` + the `znp_backend_t` interface — no ESP-IDF — so it is fully host-testable.

---

## B. Wire contract for this phase (verified vs P4 `zigbee_mgr` / TI MT)

| Dir | cmd0/cmd1 | Name | Payload |
|---|---|---|---|
| SREQ→ | 0x21/0x01 | SYS_PING | (req: none) → SRSP: 2B capabilities LE |
| SREQ→ | 0x21/0x02 | SYS_VERSION | → SRSP: `{TransportRev,Product,Major,Minor,Maint}` |
| SREQ→ | 0x21/0x04 | SYS_GET_EXTADDR | → SRSP: 8B IEEE (little-endian) |
| SREQ→ | 0x21/0x00 | SYS_RESET_REQ | side-effect: reset; **no SRSP** (P4 waits for the AREQ) |
| →AREQ | 0x41/0x80 | SYS_RESET_IND | `{Reason,TransportRev,Product,Major,Minor,Maint}`; emitted on boot |

SRSP cmd0 = `MT_SRSP(ZNP_SYS)` = 0x61; AREQ cmd0 = `MT_AREQ(ZNP_SYS)` = 0x41. P4 only logs VERSION/PING contents (not gated), so the exact version bytes are free; IEEE and the RESET_IND handshake matter. Unhandled commands → no response (silent) this phase.

---

## Task 2.1: `znp_uart` — Kconfig + init + raw TX

**Files:**
- Create: `components/znp_uart/include/znp_uart.h`
- Create: `components/znp_uart/Kconfig`
- Create: `components/znp_uart/znp_uart.c`
- Create: `components/znp_uart/CMakeLists.txt`

- [ ] **Step 1: Create `components/znp_uart/include/znp_uart.h`**

```c
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "mt_proto.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Called from the UART RX task when a complete, FCS-valid MT frame arrives.
 * frame->payload aliases an internal parser buffer (see mt_parser_feed lifetime
 * rule): consume/copy synchronously inside the callback. */
typedef void (*znp_frame_cb_t)(const mt_frame_t *frame);

/* Configure the UART (115200 8N1, no flow control, pins from Kconfig) and spawn
 * the RX task. cb is invoked per received frame. Call once at startup. */
void znp_uart_init(znp_frame_cb_t cb);

/* Write already-encoded MT bytes to the UART. Returns true if all bytes queued.
 * (Encoding lives in znp_dispatch so the wire output stays host-testable.) */
bool znp_uart_send_raw(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Create `components/znp_uart/Kconfig`**

```
menu "ZNP UART (MT NCP link)"

    config ZNP_UART_PORT
        int "UART port number"
        default 1
        help
            UART peripheral used for the MT link to the host (P4).

    config ZNP_UART_TX_GPIO
        int "TX GPIO"
        default 5

    config ZNP_UART_RX_GPIO
        int "RX GPIO"
        default 4

    config ZNP_UART_BAUD
        int "Baud rate"
        default 115200
        help
            Must match the host. P4 znp_driver uses 115200 8N1, no flow control.

endmenu
```

- [ ] **Step 3: Create `components/znp_uart/znp_uart.c` (init + TX; RX task is a stub until Task 2.2)**

```c
#include "znp_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "znp_uart";
static const uart_port_t PORT = CONFIG_ZNP_UART_PORT;
static znp_frame_cb_t s_cb = NULL;

#define ZNP_UART_RX_BUF 512

static void rx_task(void *arg);   /* implemented in Task 2.2 */

void znp_uart_init(znp_frame_cb_t cb) {
    s_cb = cb;
    const uart_config_t cfg = {
        .baud_rate = CONFIG_ZNP_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PORT, ZNP_UART_RX_BUF, ZNP_UART_RX_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PORT, CONFIG_ZNP_UART_TX_GPIO, CONFIG_ZNP_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(rx_task, "znp_uart_rx", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "UART%d up @ %d 8N1 (tx=%d rx=%d)", PORT, CONFIG_ZNP_UART_BAUD,
             CONFIG_ZNP_UART_TX_GPIO, CONFIG_ZNP_UART_RX_GPIO);
}

bool znp_uart_send_raw(const uint8_t *buf, size_t len) {
    if (!buf || len == 0) return false;
    int w = uart_write_bytes(PORT, (const char *)buf, len);
    return w == (int)len;
}

/* Placeholder so the component links in Task 2.1; replaced in Task 2.2. */
static void rx_task(void *arg) {
    (void)arg;
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
```

- [ ] **Step 4: Create `components/znp_uart/CMakeLists.txt`**

```cmake
idf_component_register(SRCS "znp_uart.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver mt_proto)
```

- [ ] **Step 5: Wire the component in temporarily and build**

Temporarily add `znp_uart` to `main/CMakeLists.txt` REQUIRES and call `znp_uart_init(NULL);` in `app_main` (this is replaced in Task 3.4 — for now it just proves the component builds and links). Edit `main/CMakeLists.txt`:
```cmake
idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash espressif__esp-zigbee-lib znp_uart)
```
And add `#include "znp_uart.h"` + `znp_uart_init(NULL);` (after the NVS init) in `app_main.c`.

Run: `source /home/user/.espressif/tools/activate_idf_v6.0.sh && idf.py -C /home/user/webapp/zhac/esp-znp-core build`
Expected: `Project build complete.`

- [ ] **Step 6: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core && git add components/znp_uart/ main/ && git commit -m "feat(znp_uart): UART init + raw TX (115200 8N1)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 2.2: `znp_uart` — RX task (parser-fed frame reassembly)

**Files:**
- Modify: `components/znp_uart/znp_uart.c` (replace the stub `rx_task`)

- [ ] **Step 1: Replace the placeholder `rx_task` in `components/znp_uart/znp_uart.c`** with the real reader (keep everything else):

```c
static void rx_task(void *arg) {
    (void)arg;
    static mt_parser_t parser;
    mt_parser_reset(&parser);
    uint8_t chunk[64];
    mt_frame_t frame;
    for (;;) {
        int n = uart_read_bytes(PORT, chunk, sizeof(chunk), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            if (mt_parser_feed(&parser, chunk[i], &frame) == 1 && s_cb) {
                s_cb(&frame);   /* payload valid only during this call */
            }
        }
    }
}
```

- [ ] **Step 2: Build**

Run: `source /home/user/.espressif/tools/activate_idf_v6.0.sh && idf.py -C /home/user/webapp/zhac/esp-znp-core build`
Expected: `Project build complete.`

- [ ] **Step 3: (Hardware) loopback smoke-test — optional but recommended**

Jumper the board's TX↔RX. Temporarily set the callback to echo: in `app_main`, pass a cb that calls `znp_uart_send_raw` after re-encoding, OR send a known frame at boot and confirm it's received. Minimal check: at boot, `znp_uart_send_raw` the PING SREQ bytes `{0xFE,0x00,0x21,0x01,0x20}`; with TX↔RX jumpered the RX task should reassemble it and the echo cb log `cmd0=0x21 cmd1=0x01`. Remove the temporary echo after verifying. (Skip if no board; this path is exercised for real in Task 3.4.)

- [ ] **Step 4: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core && git add components/znp_uart/ && git commit -m "feat(znp_uart): RX task feeds mt_parser, emits frames to callback

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 3.1: `znp_dispatch` — SYS PING/VERSION/GET_EXTADDR (host TDD)

**Files:**
- Create: `components/znp_dispatch/include/znp_dispatch.h`
- Create: `components/znp_dispatch/test/test_znp_dispatch.cpp`
- Create: `components/znp_dispatch/test/CMakeLists.txt`
- Create: `components/znp_dispatch/znp_dispatch.c`
- Create: `components/znp_dispatch/CMakeLists.txt`

- [ ] **Step 1: Create `components/znp_dispatch/include/znp_dispatch.h`**

```c
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "mt_proto.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Identity/version reported to the host. Host logs VERSION/PING (not gated),
 * so these values are free; keep them stable. */
#define ZNP_TRANSPORT_REV 0x02
#define ZNP_PRODUCT_ID    0x00
#define ZNP_VER_MAJOR     0x02
#define ZNP_VER_MINOR     0x07
#define ZNP_VER_MAINT     0x01
#define ZNP_PING_CAPS     0x0179   /* advertised MT capability bitmap (16-bit LE) */

/* Thin seam between pure protocol logic and chip-specific actions, so the
 * dispatcher is host-testable with a fake backend. */
typedef struct {
    void (*get_ieee)(uint8_t out[8]);   /* 802.15.4 EUI64, little-endian (wire order) */
    void (*request_reset)(void);        /* trigger a reset; RESET_IND follows on boot */
} znp_backend_t;

/* Handle one received request frame. If a SRSP must be sent, encodes the full
 * MT response into buf and returns its byte length (>0). Returns 0 when there is
 * no response to send (AREQ-style side-effects like RESET_REQ, or unhandled cmd). */
size_t znp_dispatch(const mt_frame_t *req, const znp_backend_t *be,
                    uint8_t *buf, size_t cap);

/* Build a SYS_RESET_IND (AREQ 0x41/0x80) into buf. reason 0x00 = power-up.
 * Returns encoded length. */
size_t znp_build_reset_ind(uint8_t reason, uint8_t *buf, size_t cap);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Write the failing test `components/znp_dispatch/test/test_znp_dispatch.cpp`**

```cpp
#include "znp_dispatch.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while(0)

/* fake backend */
static uint8_t  s_ieee[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
static int      s_reset_calls = 0;
static void fake_get_ieee(uint8_t out[8]) { memcpy(out, s_ieee, 8); }
static void fake_reset(void)              { s_reset_calls++; }
static const znp_backend_t FAKE = { fake_get_ieee, fake_reset };

static mt_frame_t req(uint8_t cmd1) { mt_frame_t f = { MT_SREQ(ZNP_SYS), cmd1, 0, nullptr }; return f; }

static void test_ping() {
    uint8_t buf[32]; mt_frame_t r = req(0x01);
    size_t n = znp_dispatch(&r, &FAKE, buf, sizeof(buf));
    const uint8_t expect[7] = {0xFE,0x02,0x61,0x01,0x79,0x01,0x1A};
    CHECK(n == 7);
    CHECK(memcmp(buf, expect, 7) == 0);
}

static void test_version() {
    uint8_t buf[32]; mt_frame_t r = req(0x02);
    size_t n = znp_dispatch(&r, &FAKE, buf, sizeof(buf));
    const uint8_t expect[10] = {0xFE,0x05,0x61,0x02,0x02,0x00,0x02,0x07,0x01,0x60};
    CHECK(n == 10);
    CHECK(memcmp(buf, expect, 10) == 0);
}

static void test_extaddr() {
    uint8_t buf[32]; mt_frame_t r = req(0x04);
    size_t n = znp_dispatch(&r, &FAKE, buf, sizeof(buf));
    const uint8_t expect[13] = {0xFE,0x08,0x61,0x04,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0xE5};
    CHECK(n == 13);
    CHECK(memcmp(buf, expect, 13) == 0);
}

static void test_unknown() {
    uint8_t buf[32]; mt_frame_t r = req(0x99);
    CHECK(znp_dispatch(&r, &FAKE, buf, sizeof(buf)) == 0);
    /* non-SYS subsystem is also unhandled this phase */
    mt_frame_t af = { MT_SREQ(ZNP_AF), 0x01, 0, nullptr };
    CHECK(znp_dispatch(&af, &FAKE, buf, sizeof(buf)) == 0);
}

int main() {
    test_ping();
    test_version();
    test_extaddr();
    test_unknown();
    if (g_fail) { printf("%d CHECK(s) failed\n", g_fail); return 1; }
    printf("all znp_dispatch tests passed\n");
    return 0;
}
```

- [ ] **Step 3: Create `components/znp_dispatch/test/CMakeLists.txt`** (compiles dispatch + the mt_proto codec it depends on)

```cmake
cmake_minimum_required(VERSION 3.16)
project(znp_dispatch_tests CXX C)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(test_znp_dispatch
    test_znp_dispatch.cpp
    ../znp_dispatch.c
    ../../mt_proto/mt_proto.cpp)
target_include_directories(test_znp_dispatch PRIVATE ../include ../../mt_proto/include)
enable_testing()
add_test(NAME znp_dispatch COMMAND test_znp_dispatch)
```

- [ ] **Step 4: Create the STUB `components/znp_dispatch/znp_dispatch.c`** (links but fails)

```c
#include "znp_dispatch.h"

size_t znp_dispatch(const mt_frame_t *req, const znp_backend_t *be,
                    uint8_t *buf, size_t cap) {
    (void)req; (void)be; (void)buf; (void)cap; return 0;
}
size_t znp_build_reset_ind(uint8_t reason, uint8_t *buf, size_t cap) {
    (void)reason; (void)buf; (void)cap; return 0;   /* implemented in Task 3.2 */
}
```

- [ ] **Step 5: Run host test — verify it FAILS**

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/znp_dispatch/test && cmake -B build -S . && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: FAIL on ping/version/extaddr (`n == 7` etc.) since the stub returns 0.

- [ ] **Step 6: Implement dispatch — replace the `znp_dispatch` body** in `components/znp_dispatch/znp_dispatch.c` (keep the `znp_build_reset_ind` stub for Task 3.2):

```c
#include "znp_dispatch.h"

static size_t encode_srsp(uint8_t cmd1, const uint8_t *pl, uint8_t pl_len,
                          uint8_t *buf, size_t cap) {
    mt_frame_t f = { MT_SRSP(ZNP_SYS), cmd1, pl_len, pl };
    return mt_encode(&f, buf, cap);
}

size_t znp_dispatch(const mt_frame_t *req, const znp_backend_t *be,
                    uint8_t *buf, size_t cap) {
    if (req->cmd0 != MT_SREQ(ZNP_SYS)) return 0;   /* only SYS this phase */
    switch (req->cmd1) {
        case 0x01: {   /* SYS_PING -> 2B capability bitmap (LE) */
            const uint8_t pl[2] = { (uint8_t)(ZNP_PING_CAPS & 0xFF),
                                    (uint8_t)(ZNP_PING_CAPS >> 8) };
            return encode_srsp(0x01, pl, 2, buf, cap);
        }
        case 0x02: {   /* SYS_VERSION */
            const uint8_t pl[5] = { ZNP_TRANSPORT_REV, ZNP_PRODUCT_ID,
                                    ZNP_VER_MAJOR, ZNP_VER_MINOR, ZNP_VER_MAINT };
            return encode_srsp(0x02, pl, 5, buf, cap);
        }
        case 0x04: {   /* SYS_GET_EXTADDR -> 8B IEEE (LE) */
            uint8_t ieee[8] = {0};
            if (be && be->get_ieee) be->get_ieee(ieee);
            return encode_srsp(0x04, ieee, 8, buf, cap);
        }
        case 0x00:     /* SYS_RESET_REQ: reset, no SRSP (host waits for RESET_IND) */
            if (be && be->request_reset) be->request_reset();
            return 0;
        default:
            return 0;  /* unhandled SYS command */
    }
}

size_t znp_build_reset_ind(uint8_t reason, uint8_t *buf, size_t cap) {
    (void)reason; (void)buf; (void)cap; return 0;   /* implemented in Task 3.2 */
}
```

- [ ] **Step 7: Run host test — verify PING/VERSION/EXTADDR/unknown PASS**

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/znp_dispatch/test && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: `all znp_dispatch tests passed`, `100% tests passed`.

- [ ] **Step 8: Create the IDF component `components/znp_dispatch/CMakeLists.txt`**

```cmake
idf_component_register(SRCS "znp_dispatch.c"
                       INCLUDE_DIRS "include"
                       REQUIRES mt_proto)
```

- [ ] **Step 9: gitignore the test build dir + verify firmware builds**

Append `components/znp_dispatch/test/build/` to `.gitignore`.
Run: `source /home/user/.espressif/tools/activate_idf_v6.0.sh && idf.py -C /home/user/webapp/zhac/esp-znp-core build` → `Project build complete.`

- [ ] **Step 10: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core && git add components/znp_dispatch/ .gitignore && git commit -m "feat(znp_dispatch): SYS ping/version/get_extaddr handlers, host-tested

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 3.2: `znp_dispatch` — RESET_REQ side-effect + `znp_build_reset_ind` (host TDD)

**Files:**
- Modify: `components/znp_dispatch/test/test_znp_dispatch.cpp` (add reset cases + calls)
- Modify: `components/znp_dispatch/znp_dispatch.c` (implement `znp_build_reset_ind`)

- [ ] **Step 1: Add failing tests** — add these two functions above `main()` and call `test_reset_req();` and `test_reset_ind();` in `main()` (after `test_unknown();`):

```cpp
static void test_reset_req() {
    uint8_t buf[32]; mt_frame_t r = req(0x00);   /* SYS_RESET_REQ */
    int before = s_reset_calls;
    CHECK(znp_dispatch(&r, &FAKE, buf, sizeof(buf)) == 0);   /* no SRSP */
    CHECK(s_reset_calls == before + 1);                      /* reset triggered */
}

static void test_reset_ind() {
    uint8_t buf[32];
    size_t n = znp_build_reset_ind(0x00, buf, sizeof(buf));
    const uint8_t expect[11] = {0xFE,0x06,0x41,0x80,0x00,0x02,0x00,0x02,0x07,0x01,0xC1};
    CHECK(n == 11);
    CHECK(memcmp(buf, expect, 11) == 0);
}
```

- [ ] **Step 2: Run — verify `test_reset_ind` FAILS** (stub returns 0; `test_reset_req` already passes since RESET_REQ was implemented in 3.1):

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/znp_dispatch/test && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: FAIL on `test_reset_ind` (`n == 11`).

- [ ] **Step 3: Implement `znp_build_reset_ind`** — replace its stub body in `znp_dispatch.c`:

```c
size_t znp_build_reset_ind(uint8_t reason, uint8_t *buf, size_t cap) {
    const uint8_t pl[6] = { reason, ZNP_TRANSPORT_REV, ZNP_PRODUCT_ID,
                            ZNP_VER_MAJOR, ZNP_VER_MINOR, ZNP_VER_MAINT };
    mt_frame_t f = { MT_AREQ(ZNP_SYS), 0x80, 6, pl };
    return mt_encode(&f, buf, cap);
}
```

- [ ] **Step 4: Run — verify ALL pass**

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/znp_dispatch/test && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: `all znp_dispatch tests passed`, `100% tests passed`.

- [ ] **Step 5: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core && git add components/znp_dispatch/ && git commit -m "feat(znp_dispatch): SYS_RESET_REQ side-effect + SYS_RESET_IND builder

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 3.3: `znp_ezb` — backend impl (on-target glue)

**Files:**
- Create: `components/znp_ezb/include/znp_ezb.h`
- Create: `components/znp_ezb/znp_ezb.c`
- Create: `components/znp_ezb/CMakeLists.txt`

- [ ] **Step 1: Create `components/znp_ezb/include/znp_ezb.h`**

```c
#pragma once
#include "znp_dispatch.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Returns the singleton backend wired to this chip (IEEE from efuse, reset = esp_restart). */
const znp_backend_t *znp_ezb_backend(void);
#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Create `components/znp_ezb/znp_ezb.c`**

```c
#include "znp_ezb.h"
#include "esp_mac.h"
#include "esp_system.h"
#include <string.h>

static void ezb_get_ieee(uint8_t out[8]) {
    uint8_t mac[8] = {0};
    /* 802.15.4 EUI64 from efuse. esp_read_mac returns it MSB-first; the MT
     * ExtAddr field is little-endian, so reverse. VERIFY this orientation on
     * first P4 integration (compare the IEEE the P4 logs vs the chip label). */
    if (esp_read_mac(mac, ESP_MAC_IEEE802154) == ESP_OK) {
        for (int i = 0; i < 8; i++) out[i] = mac[7 - i];
    } else {
        memset(out, 0, 8);   /* P4 tolerates IEEE=0 (bind falls back) */
    }
}

static void ezb_request_reset(void) {
    esp_restart();   /* reboots -> app_main emits SYS_RESET_IND on next boot */
}

const znp_backend_t *znp_ezb_backend(void) {
    static const znp_backend_t b = { ezb_get_ieee, ezb_request_reset };
    return &b;
}
```

- [ ] **Step 3: Create `components/znp_ezb/CMakeLists.txt`**

```cmake
idf_component_register(SRCS "znp_ezb.c"
                       INCLUDE_DIRS "include"
                       REQUIRES znp_dispatch esp_hw_support esp_system)
```

(`esp_read_mac` is in `esp_hw_support`; `esp_restart` in `esp_system`. If the build reports an unknown requirement, run `idf.py reconfigure` and adjust — these are the standard component names in IDF v6.0.)

- [ ] **Step 4: Build (no host test — pure on-target glue; logic is exercised via the host-tested dispatcher with the fake backend)**

Run: `source /home/user/.espressif/tools/activate_idf_v6.0.sh && idf.py -C /home/user/webapp/zhac/esp-znp-core build` → `Project build complete.`

- [ ] **Step 5: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core && git add components/znp_ezb/ && git commit -m "feat(znp_ezb): backend impl (IEEE from efuse, reset via esp_restart)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 3.4: Wire it together in `app_main` + link-up (integration)

**Files:**
- Modify: `main/CMakeLists.txt` (REQUIRES)
- Modify: `main/app_main.c` (wire UART→dispatch, emit boot RESET_IND)

- [ ] **Step 1: Update `main/CMakeLists.txt`**

```cmake
idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash espressif__esp-zigbee-lib
                                znp_uart znp_dispatch znp_ezb)
```

- [ ] **Step 2: Replace `main/app_main.c`** (keeps the NVS guards from the foundation; removes the temporary `znp_uart_init(NULL)` from Task 2.1):

```c
#include "esp_log.h"
#include "nvs_flash.h"
#include "znp_uart.h"
#include "znp_dispatch.h"
#include "znp_ezb.h"

static const char *TAG = "znp_core";

/* RX task hands each received frame here; dispatch builds an encoded response
 * (or 0) and we write it straight back. Runs in the UART RX task context. */
static void on_frame(const mt_frame_t *f) {
    uint8_t buf[260];
    size_t n = znp_dispatch(f, znp_ezb_backend(), buf, sizeof(buf));
    if (n > 0) znp_uart_send_raw(buf, n);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ret = nvs_flash_init_partition("zb_storage");
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase_partition("zb_storage"));
        ret = nvs_flash_init_partition("zb_storage");
    }
    ESP_ERROR_CHECK(ret);

    znp_uart_init(on_frame);

    /* Announce we just booted — the host toggles NRESET and waits for this. */
    uint8_t buf[16];
    size_t n = znp_build_reset_ind(0x00, buf, sizeof(buf));
    znp_uart_send_raw(buf, n);

    ESP_LOGI(TAG, "esp-znp-core MT-NCP up: SYS link ready, RESET_IND sent");
}
```

- [ ] **Step 3: Build**

Run: `source /home/user/.espressif/tools/activate_idf_v6.0.sh && idf.py -C /home/user/webapp/zhac/esp-znp-core build` → `Project build complete.`

- [ ] **Step 4: (Hardware) link-up verification** — two ways:

  **(a) Host MT smoke-test (no P4 needed).** On a workstation, with a USB-UART on the C6's TX/RX at 115200 8N1, send PING `FE 00 21 01 20` and confirm the C6 replies with a PING SRSP `FE 02 61 01 79 01 1A`; also confirm a `FE 06 41 80 ...` RESET_IND is emitted within ~1s of boot. A 10-line Python `pyserial` script suffices (write the bytes, read, compare).

  **(b) Full P4 integration.** Wire C6 TX/RX (+ optionally NRESET) to the P4's ZNP UART pins (`CONFIG_ZHAC_ZNP_UART_*`). Flash, boot both, watch the P4 monitor: it should log `SYS_RESET_IND received`, then `SYS_PING OK — alive`, then `SYS_GET_EXTADDR` succeeding (or the IEEE=0 fallback warning). Reaching "PING OK / alive" = **link-up achieved.**

  Expected: P4 progresses past chip-detection. If the P4 logs `SYS_PING failed`, check baud/pins/wiring and FCS (host tests already prove the bytes; a wire fault is the likely cause). If IEEE looks byte-reversed, fix the `ezb_get_ieee` ordering (Task 3.3 flagged this).

- [ ] **Step 5: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core && git add main/ && git commit -m "feat: wire UART->dispatch + emit boot SYS_RESET_IND (P4 link-up)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## C. Self-Review (run against this plan)

- **Spec coverage:** Phase 2 UART transport — init+TX (2.1), RX-parser task (2.2). Phase 3 — SYS PING/VERSION/GET_EXTADDR (3.1), RESET_REQ + RESET_IND builder (3.2), chip backend (3.3), wiring + boot RESET_IND + link-up (3.4). Covers every SYS row + the RESET_IND AREQ from §B.
- **Placeholder scan:** all steps contain full code. The only staged stubs are the intentional TDD stubs (`rx_task` in 2.1→2.2; `znp_dispatch`/`znp_build_reset_ind` in 3.1→3.2) — each completed in a named later step.
- **Type consistency:** `znp_backend_t` (`get_ieee`/`request_reset`) identical across header, fake (tests), and `znp_ezb`. `znp_dispatch`/`znp_build_reset_ind` signatures match between header, stub, impl, and tests. `znp_frame_cb_t` matches `znp_uart_init` and `on_frame`. Constants (`ZNP_*`, `MT_SREQ/SRSP/AREQ`, `ZNP_SYS`) reused, not redefined.
- **Test-vector arithmetic (hand-verified):** PING SRSP FCS `2^0x61^0x01^0x79^0x01 = 0x1A`; VERSION SRSP FCS `5^0x61^0x02^0x02^0x00^0x02^0x07^0x01 = 0x60`; GET_EXTADDR SRSP FCS `8^0x61^0x04^(0x11^…^0x88=0x88) = 0xE5`; RESET_IND FCS `0xC1` (matches the parser test vector in the foundation plan). All consistent with `mt_fcs`.
- **Host-testability:** `znp_dispatch` + `znp_build_reset_ind` are pure (mt_proto + fn-ptr backend) → real TDD. UART/ezb are thin glue verified by build + on-target steps; their only logic (frame reassembly) is already covered by `mt_proto` parser tests.

## D. Open items / on-hardware verification (not blockers for the code)
- **IEEE byte order** in `ezb_get_ieee` (Task 3.3) — confirm against what the P4 logs on first integration; flip the loop if reversed.
- **UART pins** (`CONFIG_ZNP_UART_TX/RX_GPIO` defaults 5/4) — set to the actual C6/H2↔P4 wiring.
- **NRESET handshake / reset-burst:** the P4 toggles NRESET and has reset-burst detection. This phase emits RESET_IND once per boot (correct). Don't emit spurious RESET_INDs. If the P4 drives NRESET, ensure the C6 reset line is wired so a P4-initiated reset reboots the C6 (which then emits RESET_IND) — otherwise rely on `SYS_RESET_REQ`→`esp_restart`.
- **`esp_read_mac` requirement name** (`esp_hw_support`) — verify on `idf.py reconfigure`.
