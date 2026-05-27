# esp-znp-core — MT-NCP Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up the `esp-znp-core` ESP-IDF project (ESP32-C6/H2) and a host-tested MT (Monitor & Test) wire codec that is byte-identical to the P4's `znp_driver`, so the C6/H2 can later act as a TI-ZNP-compatible NCP that `zhac-main-core` talks to unchanged.

**Architecture:** C6/H2 runs esp-zigbee-lib v2.x as a Zigbee coordinator; a thin firmware translates the TI MT wire protocol (over UART) ↔ `ezb_*`/`esp_zigbee_*` calls. This plan builds only the **foundation**: a booting IDF skeleton + a standalone, host-tested MT frame codec component (`mt_proto`). No Zigbee stack start, no UART wiring yet — those are later plans (see Roadmap §B).

**Tech Stack:** ESP-IDF v6.0, esp-zigbee-lib ≥2.0.0 (component registry), C++17 for logic components (host-testable), C for `app_main`, CMake/CTest for host tests. Target `esp32c6` (swap to `esp32h2` via `idf.py set-target`).

**Design decision — why a self-contained codec component:** the MT framing (`SOF 0xFE`, `LEN`, `CMD0`, `CMD1`, payload, `FCS = XOR`) is the wire contract with the P4. It is pure logic with zero ESP-IDF dependency, so it lives in its own component compiled **both** into the firmware and into a host CTest binary. Wire bugs get caught on the host in milliseconds, not on hardware.

---

## A. Whole-Project File Structure (target end-state, for context)

```
esp-znp-core/
├── CMakeLists.txt                 # IDF top-level project
├── sdkconfig.defaults             # target + zigbee + custom partition table
├── partitions.csv                 # zb_storage = NVS subtype (v2.x requirement)
├── docs/superpowers/plans/        # this plan + future plans
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml           # dep: espressif/esp-zigbee-lib >=2.0.0
│   └── app_main.c                  # boot: NVS init + version banner (Phase 0)
└── components/
    ├── mt_proto/                   # ← THIS PLAN. MT wire codec. Host-testable.
    │   ├── include/mt_proto.h
    │   ├── mt_proto.cpp
    │   ├── CMakeLists.txt           # IDF component registration
    │   └── test/                    # standalone host CMake + CTest
    │       ├── CMakeLists.txt
    │       └── test_mt_proto.cpp
    ├── znp_uart/                   # (Roadmap Phase 2) UART transport + RX framer
    ├── znp_dispatch/               # (Roadmap Phase 3) (cmd0,cmd1)→ZnpBackend, host-tested
    └── znp_ezb/                    # (Roadmap Phase 3+) ZnpBackend impl via ezb_*/esp_zigbee_*
```

**This plan creates only:** the root IDF files, `main/`, and `components/mt_proto/` (+ its host test). Everything else is later plans.

---

## B. Roadmap (milestones — each later phase becomes its own plan)

| Phase | Milestone | Produces | Test surface |
|---|---|---|---|
| **0 (this plan)** | Project boots | C6/H2 IDF skeleton, esp-zigbee-lib links, prints version | flash + serial log |
| **1 (this plan)** | MT codec correct | `mt_proto` encode/decode/streaming-parser, byte-identical to `znp_driver` | **host CTest** |
| 2 (next plan) | Frames cross UART | `znp_uart`: 115200 8N1 RX task → parser → frame queue; TX | HW loopback / P4 |
| 3 (next plan) | **P4 sees NCP alive** | `znp_dispatch` (host-tested w/ fake backend) + `znp_ezb` SYS handlers; boot emits `SYS_RESET_IND`; answers `SYS_PING/VERSION/GET_EXTADDR`; `SYS_RESET_REQ` | host + P4 link-up |
| 4 | Network forms | NV-write buffering→`ezb_set_*`; `ZDO_STARTUP_FROM_APP`+`APP_CNF` BDB→`ezb_bdb_*`; `AF_REGISTER`; `STATE_CHANGE_IND` | P4 "network up" |
| 5 | Device joins + interview | `ZDO_*_REQ`→`ezb_zdo_*`; RSP/IND AREQs; bind/unbind/leave/permit-join | real device |
| 6 | End-to-end control | `AF_DATA_REQUEST`→`ezb_apsde_data_request`; APSDE indication→`AF_INCOMING_MSG`; confirm | device via ZHC |

Wire contract + per-command mapping references (read before Phases 3–6):
`../../../ZNP_API_SURFACE_C6_PORT.md`, `../../../ESP_ZIGBEE_SDK_V2_NOTES.md` (rename map + §8 APS), `../../../C6_BACKEND_PATH_B_SCOPE.md`.

---

## Task 0.1: IDF project skeleton (boots + links esp-zigbee-lib)

**Files:**
- Create: `CMakeLists.txt`
- Create: `sdkconfig.defaults`
- Create: `partitions.csv`
- Create: `main/CMakeLists.txt`
- Create: `main/idf_component.yml`
- Create: `main/app_main.c`

- [ ] **Step 1: Create top-level `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp-znp-core)
```

- [ ] **Step 2: Create `sdkconfig.defaults`**

```ini
CONFIG_IDF_TARGET="esp32c6"
# esp-zigbee-lib (v2.x) — native 802.15.4 radio on C6/H2
CONFIG_ZB_ENABLED=y
CONFIG_ZB_RADIO_NATIVE=y
# custom partition table (zb_storage must be NVS in v2.x)
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

- [ ] **Step 3: Create `partitions.csv`** (zb_storage subtype = `nvs`, per esp-zigbee-sdk v2.x migration)

```csv
# Name,       Type, SubType, Offset,  Size,    Flags
nvs,          data, nvs,     ,        0x6000,
phy_init,     data, phy,     ,        0x1000,
factory,      app,  factory, ,        0x140000,
zb_storage,   data, nvs,     ,        0x4000,
zb_fct,       data, nvs,     ,        0x1000,
```

Note: sizes mirror the esp-zigbee HA examples; if `idf.py build` complains the table exceeds flash, shrink `factory`. The **critical** line is `zb_storage … nvs` (v2.x requires NVS, not FAT).

- [ ] **Step 4: Create `main/idf_component.yml`**

```yaml
dependencies:
  idf:
    version: ">=5.0"
  espressif/esp-zigbee-lib:
    version: ">=2.0.0"
```

- [ ] **Step 5: Create `main/CMakeLists.txt`**

```cmake
idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash espressif__esp-zigbee-lib)
```

Note: the managed dependency `espressif/esp-zigbee-lib` is referenced in `REQUIRES` as `espressif__esp-zigbee-lib` (registry namespacing uses `__`). If the build reports it unknown, run `idf.py reconfigure` and use the exact name printed under `managed_components/`.

- [ ] **Step 6: Create `main/app_main.c`**

```c
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee.h"   // v2.x all-in-one header (was esp_zigbee_core.h in v1.x)

static const char *TAG = "znp_core";

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    // v2.x: zb_storage is an NVS partition the stack will use later.
    ESP_ERROR_CHECK(nvs_flash_init_partition("zb_storage"));

    ESP_LOGI(TAG, "esp-znp-core boot");
    ESP_LOGI(TAG, "esp-zigbee-lib version: %s", esp_zigbee_get_version_string());
    ESP_LOGI(TAG, "MT-NCP foundation ready (stack not started)");
}
```

Note: if `esp_zigbee_get_version_string` is unresolved, the installed lib may expose it under a different symbol — grep `managed_components/espressif__esp-zigbee-lib/include` for `version_string` and adjust. This call exists only as a link smoke-test.

- [ ] **Step 7: Set target and build**

Run: `cd /home/user/webapp/zhac/esp-znp-core && source ../../activate_idf_v6.0.sh && idf.py set-target esp32c6 && idf.py build`
Expected: `Project build complete.` — esp-zigbee-lib downloaded into `managed_components/`, firmware links, no errors.

- [ ] **Step 8: (Hardware, optional) flash + observe boot**

Run: `idf.py -p /dev/ttyACM0 flash monitor`
Expected serial log: `esp-znp-core boot`, `esp-zigbee-lib version: …`, `MT-NCP foundation ready`. (Skip if no board attached; build success in Step 7 is the gating criterion.)

- [ ] **Step 9: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core
git init 2>/dev/null; git add CMakeLists.txt sdkconfig.defaults partitions.csv main/ docs/
git commit -m "feat: esp-znp-core IDF skeleton, links esp-zigbee-lib v2.x"
```

---

## Task 1.1: `mt_proto` — FCS + encode (host TDD)

**Files:**
- Create: `components/mt_proto/include/mt_proto.h`
- Create: `components/mt_proto/test/test_mt_proto.cpp`
- Create: `components/mt_proto/test/CMakeLists.txt`
- Create: `components/mt_proto/mt_proto.cpp`
- Create: `components/mt_proto/CMakeLists.txt`

- [ ] **Step 1: Create the public header `components/mt_proto/include/mt_proto.h`**

```cpp
#pragma once
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MT_SOF          0xFE
#define MT_MAX_PAYLOAD  250
#define MT_OVERHEAD     5     /* SOF + LEN + CMD0 + CMD1 + FCS */

/* type|subsystem helpers — identical to znp_driver */
#define MT_SREQ(sub)  ((uint8_t)(0x20 | (sub)))
#define MT_AREQ(sub)  ((uint8_t)(0x40 | (sub)))
#define MT_SRSP(sub)  ((uint8_t)(0x60 | (sub)))
#define ZNP_SYS       0x01
#define ZNP_AF        0x04
#define ZNP_ZDO       0x05
#define ZNP_UTIL      0x07
#define ZNP_APP_CNF   0x0F

typedef struct {
    uint8_t        cmd0;
    uint8_t        cmd1;
    uint8_t        payload_len;
    const uint8_t *payload;       /* non-owning */
} mt_frame_t;

typedef enum {
    MT_DECODE_OK = 0,
    MT_DECODE_BAD_SOF,
    MT_DECODE_FCS_ERROR,
    MT_DECODE_TRUNCATED,
    MT_DECODE_OVERFLOW,
} mt_decode_result_t;

uint8_t mt_fcs(uint8_t len, uint8_t cmd0, uint8_t cmd1,
               const uint8_t *payload, uint8_t payload_len);

/* Encode frame into buf. Returns total bytes written, or 0 on overflow/too-small buf. */
size_t  mt_encode(const mt_frame_t *f, uint8_t *buf, size_t buf_size);

/* Decode one complete buffer (buf[0]==SOF .. includes trailing FCS).
 * On OK, out->payload points into buf+4. */
mt_decode_result_t mt_decode(const uint8_t *buf, size_t len, mt_frame_t *out);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Write the failing test `components/mt_proto/test/test_mt_proto.cpp`**

```cpp
#include "mt_proto.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while(0)

static void test_fcs() {
    /* SYS_PING SREQ: len0 cmd0 0x21 cmd1 0x01 -> 0x00^0x21^0x01 = 0x20 */
    CHECK(mt_fcs(0, 0x21, 0x01, nullptr, 0) == 0x20);
    const uint8_t pl[2] = {0x79, 0x00};
    /* len2 cmd0 0x61 cmd1 0x01 -> 2^0x61^0x01^0x79^0x00 = 0x1B */
    CHECK(mt_fcs(2, 0x61, 0x01, pl, 2) == 0x1B);
}

static void test_encode() {
    uint8_t buf[16];
    mt_frame_t ping = { MT_SREQ(ZNP_SYS), 0x01, 0, nullptr };   /* 0x21,0x01 */
    size_t n = mt_encode(&ping, buf, sizeof(buf));
    CHECK(n == 5);
    const uint8_t expect[5] = {0xFE, 0x00, 0x21, 0x01, 0x20};
    CHECK(memcmp(buf, expect, 5) == 0);

    /* buf too small -> 0 */
    uint8_t tiny[3];
    CHECK(mt_encode(&ping, tiny, sizeof(tiny)) == 0);
}

int main() {
    test_fcs();
    test_encode();
    if (g_fail) { printf("%d CHECK(s) failed\n", g_fail); return 1; }
    printf("all mt_proto tests passed\n");
    return 0;
}
```

- [ ] **Step 3: Create the host test CMake `components/mt_proto/test/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
project(mt_proto_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(test_mt_proto test_mt_proto.cpp ../mt_proto.cpp)
target_include_directories(test_mt_proto PRIVATE ../include)
enable_testing()
add_test(NAME mt_proto COMMAND test_mt_proto)
```

- [ ] **Step 4: Create a stub `components/mt_proto/mt_proto.cpp` so it links but fails**

```cpp
#include "mt_proto.h"

uint8_t mt_fcs(uint8_t, uint8_t, uint8_t, const uint8_t *, uint8_t) { return 0; }
size_t  mt_encode(const mt_frame_t *, uint8_t *, size_t) { return 0; }
mt_decode_result_t mt_decode(const uint8_t *, size_t, mt_frame_t *) { return MT_DECODE_TRUNCATED; }
```

- [ ] **Step 5: Run the host test — verify it FAILS**

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/mt_proto/test && cmake -B build -S . && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: FAIL — `FAIL …  mt_fcs(0, 0x21, 0x01, nullptr, 0) == 0x20` etc.

- [ ] **Step 6: Implement `mt_fcs` + `mt_encode` in `components/mt_proto/mt_proto.cpp`** (replace stub body for these two; keep `mt_decode` stub for now)

```cpp
#include "mt_proto.h"

uint8_t mt_fcs(uint8_t len, uint8_t cmd0, uint8_t cmd1,
               const uint8_t *payload, uint8_t payload_len) {
    uint8_t f = (uint8_t)(len ^ cmd0 ^ cmd1);
    for (uint8_t i = 0; i < payload_len; i++) f ^= payload[i];
    return f;
}

size_t mt_encode(const mt_frame_t *f, uint8_t *buf, size_t buf_size) {
    if (f->payload_len > MT_MAX_PAYLOAD) return 0;
    const size_t total = (size_t)MT_OVERHEAD + f->payload_len;
    if (buf_size < total) return 0;
    buf[0] = MT_SOF;
    buf[1] = f->payload_len;
    buf[2] = f->cmd0;
    buf[3] = f->cmd1;
    for (uint8_t i = 0; i < f->payload_len; i++) buf[4 + i] = f->payload[i];
    buf[4 + f->payload_len] = mt_fcs(f->payload_len, f->cmd0, f->cmd1,
                                     f->payload, f->payload_len);
    return total;
}

mt_decode_result_t mt_decode(const uint8_t *, size_t, mt_frame_t *) {
    return MT_DECODE_TRUNCATED;   /* implemented in Task 1.2 */
}
```

- [ ] **Step 7: Run the host test — verify FCS + encode PASS**

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/mt_proto/test && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: `all mt_proto tests passed`, CTest `100% tests passed`.

- [ ] **Step 8: Create the IDF component registration `components/mt_proto/CMakeLists.txt`**

```cmake
idf_component_register(SRCS "mt_proto.cpp"
                       INCLUDE_DIRS "include")
```

- [ ] **Step 9: Verify the component cross-compiles into firmware**

Run: `cd /home/user/webapp/zhac/esp-znp-core && idf.py build`
Expected: `Project build complete.` (mt_proto compiles for esp32c6 too.)

- [ ] **Step 10: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core
git add components/mt_proto/
git commit -m "feat(mt_proto): MT FCS + encode with host tests"
```

---

## Task 1.2: `mt_proto` — `mt_decode` (host TDD)

**Files:**
- Modify: `components/mt_proto/test/test_mt_proto.cpp` (add decode cases + call from main)
- Modify: `components/mt_proto/mt_proto.cpp` (implement `mt_decode`)

- [ ] **Step 1: Add the failing decode test** — insert this function above `main()` in `test_mt_proto.cpp`, and add `test_decode();` as the third line of `main()`:

```cpp
static void test_decode() {
    mt_frame_t out;
    /* valid SYS_PING SREQ */
    const uint8_t ping[5] = {0xFE, 0x00, 0x21, 0x01, 0x20};
    CHECK(mt_decode(ping, 5, &out) == MT_DECODE_OK);
    CHECK(out.cmd0 == 0x21 && out.cmd1 == 0x01 && out.payload_len == 0);

    /* valid SYS_PING SRSP with 2-byte payload */
    const uint8_t srsp[7] = {0xFE, 0x02, 0x61, 0x01, 0x79, 0x00, 0x1B};
    CHECK(mt_decode(srsp, 7, &out) == MT_DECODE_OK);
    CHECK(out.payload_len == 2 && out.payload[0] == 0x79 && out.payload[1] == 0x00);

    /* bad SOF */
    const uint8_t bad_sof[5] = {0x00, 0x00, 0x21, 0x01, 0x20};
    CHECK(mt_decode(bad_sof, 5, &out) == MT_DECODE_BAD_SOF);

    /* corrupted FCS */
    const uint8_t bad_fcs[5] = {0xFE, 0x00, 0x21, 0x01, 0xFF};
    CHECK(mt_decode(bad_fcs, 5, &out) == MT_DECODE_FCS_ERROR);

    /* truncated */
    CHECK(mt_decode(ping, 3, &out) == MT_DECODE_TRUNCATED);
}
```

- [ ] **Step 2: Run — verify decode cases FAIL**

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/mt_proto/test && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: FAIL on the decode CHECKs (stub returns TRUNCATED).

- [ ] **Step 3: Implement `mt_decode`** — replace the `mt_decode` stub body in `mt_proto.cpp`:

```cpp
mt_decode_result_t mt_decode(const uint8_t *buf, size_t len, mt_frame_t *out) {
    if (len < MT_OVERHEAD)                  return MT_DECODE_TRUNCATED;
    if (buf[0] != MT_SOF)                   return MT_DECODE_BAD_SOF;
    const uint8_t plen = buf[1];
    if (plen > MT_MAX_PAYLOAD)              return MT_DECODE_OVERFLOW;
    if (len < (size_t)MT_OVERHEAD + plen)   return MT_DECODE_TRUNCATED;
    const uint8_t expect = mt_fcs(plen, buf[2], buf[3], buf + 4, plen);
    if (expect != buf[4 + plen])            return MT_DECODE_FCS_ERROR;
    out->cmd0        = buf[2];
    out->cmd1        = buf[3];
    out->payload_len = plen;
    out->payload     = buf + 4;
    return MT_DECODE_OK;
}
```

- [ ] **Step 4: Run — verify ALL pass**

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/mt_proto/test && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: `all mt_proto tests passed`, `100% tests passed`.

- [ ] **Step 5: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core
git add components/mt_proto/
git commit -m "feat(mt_proto): mt_decode with FCS/SOF/truncation tests"
```

---

## Task 1.3: `mt_proto` — streaming parser (host TDD)

The UART feeds bytes one at a time; the parser reassembles frames. State machine mirrors `znp_driver`'s `znp_parser.cpp` (Sof→Len→Cmd0→Cmd1→Data→Fcs).

**Files:**
- Modify: `components/mt_proto/include/mt_proto.h` (add parser API)
- Modify: `components/mt_proto/test/test_mt_proto.cpp` (add parser cases + call)
- Modify: `components/mt_proto/mt_proto.cpp` (implement parser)

- [ ] **Step 1: Add parser API to `mt_proto.h`** — insert before the closing `#ifdef __cplusplus`/`}`:

```cpp
typedef struct {
    uint8_t state;                  /* 0=SOF 1=LEN 2=CMD0 3=CMD1 4=DATA 5=FCS */
    uint8_t len;
    uint8_t cmd0;
    uint8_t cmd1;
    uint8_t idx;
    uint8_t data[MT_MAX_PAYLOAD];
} mt_parser_t;

void mt_parser_reset(mt_parser_t *p);
/* Feed one byte. Returns 1 and fills *out (out->payload -> p->data) when a full,
 * FCS-valid frame completes; returns 0 otherwise (incl. FCS mismatch -> resets). */
int  mt_parser_feed(mt_parser_t *p, uint8_t b, mt_frame_t *out);
```

- [ ] **Step 2: Add the failing parser test** — add above `main()` and call `test_parser();` in `main()`:

```cpp
static void test_parser() {
    mt_parser_t p; mt_parser_reset(&p);
    mt_frame_t out;
    const uint8_t ping[5] = {0xFE, 0x00, 0x21, 0x01, 0x20};

    /* leading garbage (non-SOF) is ignored */
    CHECK(mt_parser_feed(&p, 0x11, &out) == 0);
    /* feed the 5 frame bytes; only the last (FCS) byte completes a frame */
    CHECK(mt_parser_feed(&p, ping[0], &out) == 0);
    CHECK(mt_parser_feed(&p, ping[1], &out) == 0);
    CHECK(mt_parser_feed(&p, ping[2], &out) == 0);
    CHECK(mt_parser_feed(&p, ping[3], &out) == 0);
    CHECK(mt_parser_feed(&p, ping[4], &out) == 1);
    CHECK(out.cmd0 == 0x21 && out.cmd1 == 0x01 && out.payload_len == 0);

    /* a frame with payload: SYS_RESET_IND AREQ 0x41/0x80, 6-byte payload */
    const uint8_t ind[11] = {0xFE,0x06,0x41,0x80,0x00,0x02,0x00,0x02,0x07,0x01,0xC1};
    int done = 0;
    for (int i = 0; i < 11; i++) done = mt_parser_feed(&p, ind[i], &out);
    CHECK(done == 1);
    CHECK(out.cmd0 == 0x41 && out.cmd1 == 0x80 && out.payload_len == 6);
    CHECK(out.payload[1] == 0x02 && out.payload[4] == 0x07);

    /* bad FCS resets without emitting */
    mt_parser_reset(&p);
    const uint8_t badf[5] = {0xFE,0x00,0x21,0x01,0xFF};
    for (int i = 0; i < 4; i++) CHECK(mt_parser_feed(&p, badf[i], &out) == 0);
    CHECK(mt_parser_feed(&p, badf[4], &out) == 0);   /* FCS mismatch -> 0 */
}
```

- [ ] **Step 3: Run — verify parser cases FAIL (undefined refs)**

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/mt_proto/test && cmake -B build -S . && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: link error `undefined reference to mt_parser_feed` / `mt_parser_reset` (clean reconfigure needed because a new symbol was referenced).

- [ ] **Step 4: Implement the parser** — append to `mt_proto.cpp`:

```cpp
void mt_parser_reset(mt_parser_t *p) {
    p->state = 0; p->len = 0; p->idx = 0;
}

int mt_parser_feed(mt_parser_t *p, uint8_t b, mt_frame_t *out) {
    switch (p->state) {
        case 0: if (b == MT_SOF) p->state = 1; return 0;            /* SOF */
        case 1:                                                     /* LEN */
            p->len = b;
            if (p->len > MT_MAX_PAYLOAD) { mt_parser_reset(p); return 0; }
            p->state = 2; return 0;
        case 2: p->cmd0 = b; p->state = 3; return 0;                /* CMD0 */
        case 3: p->cmd1 = b; p->idx = 0; p->state = (p->len ? 4 : 5); return 0; /* CMD1 */
        case 4:                                                     /* DATA */
            p->data[p->idx++] = b;
            if (p->idx >= p->len) p->state = 5;
            return 0;
        case 5: {                                                   /* FCS */
            const uint8_t expect = mt_fcs(p->len, p->cmd0, p->cmd1, p->data, p->len);
            const int ok = (expect == b);
            if (ok && out) {
                out->cmd0 = p->cmd0; out->cmd1 = p->cmd1;
                out->payload_len = p->len; out->payload = p->data;
            }
            mt_parser_reset(p);
            return ok ? 1 : 0;
        }
        default: mt_parser_reset(p); return 0;
    }
}
```

- [ ] **Step 5: Run — verify ALL pass**

Run: `cd /home/user/webapp/zhac/esp-znp-core/components/mt_proto/test && cmake --build build && ctest --test-dir build --output-on-failure`
Expected: `all mt_proto tests passed`, `100% tests passed`.

- [ ] **Step 6: Verify firmware still builds**

Run: `cd /home/user/webapp/zhac/esp-znp-core && idf.py build`
Expected: `Project build complete.`

- [ ] **Step 7: Commit**

```bash
cd /home/user/webapp/zhac/esp-znp-core
git add components/mt_proto/
git commit -m "feat(mt_proto): streaming frame parser with host tests"
```

---

## C. Self-Review (run against this plan)

- **Spec coverage (foundation scope):** Phase 0 skeleton ✔ (Task 0.1); Phase 1 codec — FCS ✔, encode ✔ (1.1), decode ✔ (1.2), streaming parser ✔ (1.3). Phases 2–6 explicitly deferred to their own plans (Roadmap §B) per the scope-check.
- **Placeholder scan:** every code step contains full file/function bodies; no TBD/TODO in tasks. The only "implemented later" marker is the deliberate `mt_decode` stub in Task 1.1 Step 6, completed in Task 1.2 Step 3 (standard TDD staging).
- **Type consistency:** `mt_frame_t` fields (`cmd0`,`cmd1`,`payload_len`,`payload`) used identically across header, tests, encode, decode, parser. `mt_decode_result_t` enumerators match between header and tests. Constants `MT_SOF`/`MT_OVERHEAD`/`MT_MAX_PAYLOAD`/`MT_SREQ`/`ZNP_SYS` consistent. Test vectors hand-verified: PING SREQ FCS=0x20, PING SRSP FCS=0x1B, RESET_IND FCS=0xC1.
- **Wire-contract fidelity:** framing/FCS copied from `znp_driver` (`SOF 0xFE`, `FCS = LEN^CMD0^CMD1^payload`, overhead 5, max payload 250) — guarantees P4 byte-compatibility, which the host tests lock in.

## D. Open items to resolve at Phase-2/3 planning time (not this plan)
- Exact UART pins on the C6/H2 board + whether NRESET line is wired (drives `SYS_RESET_IND` handshake).
- `esp_zigbee_get_version_string` symbol name on the pinned lib (smoke-tested in Task 0.1 Step 6).
- `znp_dispatch` ⇄ `ZnpBackend` interface shape (host-testable with a fake backend) — design in the Phase-3 plan.
