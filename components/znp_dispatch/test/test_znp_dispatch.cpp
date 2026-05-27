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
