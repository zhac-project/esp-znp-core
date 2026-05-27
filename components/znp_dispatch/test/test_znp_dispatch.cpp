#include "znp_dispatch.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while(0)

/* ── fake backend ────────────────────────────────────────────────────────── */
static uint8_t  s_ieee[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
static int      s_reset_calls = 0;
static void fake_get_ieee(uint8_t out[8]) { memcpy(out, s_ieee, 8); }
static void fake_reset(void)              { s_reset_calls++; }
static const znp_backend_t FAKE = { fake_get_ieee, fake_reset };

/* Helper: fresh ctx wrapping FAKE each time */
static znp_dispatch_ctx make_ctx(void) {
    znp_dispatch_ctx c;
    memset(&c, 0, sizeof(c));
    c.be = &FAKE;
    return c;
}

static mt_frame_t req(uint8_t cmd1) {
    mt_frame_t f = { MT_SREQ(ZNP_SYS), cmd1, 0, nullptr };
    return f;
}

/* ── existing tests (updated to ctx form) ───────────────────────────────── */

static void test_ping() {
    uint8_t buf[32]; mt_frame_t r = req(0x01);
    znp_dispatch_ctx ctx = make_ctx();
    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));
    const uint8_t expect[7] = {0xFE,0x02,0x61,0x01,0x79,0x01,0x1A};
    CHECK(n == 7);
    CHECK(memcmp(buf, expect, 7) == 0);
}

static void test_version() {
    uint8_t buf[32]; mt_frame_t r = req(0x02);
    znp_dispatch_ctx ctx = make_ctx();
    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));
    const uint8_t expect[10] = {0xFE,0x05,0x61,0x02,0x02,0x00,0x02,0x07,0x01,0x60};
    CHECK(n == 10);
    CHECK(memcmp(buf, expect, 10) == 0);
}

static void test_extaddr() {
    uint8_t buf[32]; mt_frame_t r = req(0x04);
    znp_dispatch_ctx ctx = make_ctx();
    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));
    const uint8_t expect[13] = {0xFE,0x08,0x61,0x04,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0xE5};
    CHECK(n == 13);
    CHECK(memcmp(buf, expect, 13) == 0);
}

static void test_unknown() {
    uint8_t buf[32];
    znp_dispatch_ctx ctx = make_ctx();
    mt_frame_t r = req(0x99);
    CHECK(znp_dispatch(&r, &ctx, buf, sizeof(buf)) == 0);
    /* non-SYS subsystem is also unhandled this phase */
    mt_frame_t af = { MT_SREQ(ZNP_AF), 0x01, 0, nullptr };
    CHECK(znp_dispatch(&af, &ctx, buf, sizeof(buf)) == 0);
}

static void test_reset_req() {
    uint8_t buf[32]; mt_frame_t r = req(0x00);   /* SYS_RESET_REQ */
    znp_dispatch_ctx ctx = make_ctx();
    int before = s_reset_calls;
    CHECK(znp_dispatch(&r, &ctx, buf, sizeof(buf)) == 0);   /* no SRSP */
    CHECK(s_reset_calls == before + 1);                     /* reset triggered */
}

static void test_reset_ind() {
    uint8_t buf[32];
    size_t n = znp_build_reset_ind(0x00, buf, sizeof(buf));
    const uint8_t expect[11] = {0xFE,0x06,0x41,0x80,0x00,0x02,0x00,0x02,0x07,0x01,0xC1};
    CHECK(n == 11);
    CHECK(memcmp(buf, expect, 11) == 0);
}

/* ── new tests: znp_netcfg_apply_nv (pure unit) ─────────────────────────── */

static void test_netcfg(void) {
    znp_netcfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* PANID: 2-byte LE {0x34, 0x12} -> pan_id == 0x1234 */
    const uint8_t panid_val[2] = {0x34, 0x12};
    CHECK(znp_netcfg_apply_nv(&cfg, ZNP_NV_PANID, panid_val, 2) == true);
    CHECK(cfg.pan_id == 0x1234);
    CHECK(cfg.have_pan_id == true);

    /* CHANLIST: channel-15 mask 0x00008000, LE bytes = {0x00,0x80,0x00,0x00} */
    const uint8_t chan_val[4] = {0x00, 0x80, 0x00, 0x00};
    CHECK(znp_netcfg_apply_nv(&cfg, ZNP_NV_CHANLIST, chan_val, 4) == true);
    CHECK(cfg.chan_mask == 0x00008000u);
    CHECK(cfg.have_chan_mask == true);

    /* PRECFGKEY: 16-byte network key */
    const uint8_t key_val[16] = {
        0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
        0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10
    };
    CHECK(znp_netcfg_apply_nv(&cfg, ZNP_NV_PRECFGKEY, key_val, 16) == true);
    CHECK(memcmp(cfg.nwk_key, key_val, 16) == 0);
    CHECK(cfg.have_nwk_key == true);

    /* LOGICAL_TYPE: coordinator = 0x00 */
    const uint8_t ltype_val[1] = {0x00};
    CHECK(znp_netcfg_apply_nv(&cfg, ZNP_NV_LOGICAL_TYPE, ltype_val, 1) == true);
    CHECK(cfg.logical_type == 0x00);
    CHECK(cfg.have_logical_type == true);

    /* Unknown NV id: should be accepted (true) and not disturb cfg */
    uint16_t old_pan = cfg.pan_id;
    const uint8_t unk_val[4] = {0xAA,0xBB,0xCC,0xDD};
    CHECK(znp_netcfg_apply_nv(&cfg, 0x1234, unk_val, 4) == true);
    CHECK(cfg.pan_id == old_pan);   /* unchanged */

    /* Short val guard: PANID with len<2 must not crash or set flag */
    znp_netcfg_t cfg2;
    memset(&cfg2, 0, sizeof(cfg2));
    const uint8_t short_val[1] = {0xFF};
    CHECK(znp_netcfg_apply_nv(&cfg2, ZNP_NV_PANID, short_val, 1) == true);
    CHECK(cfg2.have_pan_id == false);
}

/* ── new test: NV_WRITE_EXT dispatch round-trip ─────────────────────────── */

static void test_nv_write_dispatch(void) {
    uint8_t buf[32];

    /* Build a SYS_OSAL_NV_WRITE_EXT frame exactly as P4 nv_write_raw does:
     * cmd0=MT_SREQ(ZNP_SYS)=0x21, cmd1=0x1D
     * payload = id(2 LE) + offset(2 LE) + len(2 LE) + data[len]
     * For PANID=0x1234: id={0x83,0x00}, offset={0x00,0x00}, len={0x02,0x00},
     *                   data={0x34,0x12}
     */
    const uint8_t payload[8] = {
        0x83, 0x00,   /* id = 0x0083 = NV_PANID, LE */
        0x00, 0x00,   /* offset = 0, LE */
        0x02, 0x00,   /* len = 2, LE */
        0x34, 0x12    /* data: PAN ID 0x1234, LE */
    };
    mt_frame_t r = { MT_SREQ(ZNP_SYS), 0x1D, sizeof(payload), payload };

    znp_dispatch_ctx ctx = make_ctx();
    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));

    /* Response must be a SRSP (cmd0=0x61, cmd1=0x1D) with status=0x00 */
    /* FE + len(1) + cmd0(1) + cmd1(1) + status(1) + fcs(1) = 6 bytes */
    CHECK(n == 6);
    CHECK(buf[0] == 0xFE);
    CHECK(buf[1] == 0x01);       /* payload len */
    CHECK(buf[2] == MT_SRSP(ZNP_SYS));  /* 0x61 */
    CHECK(buf[3] == 0x1D);
    CHECK(buf[4] == 0x00);       /* status = success */

    /* NV write must have been buffered into ctx.cfg */
    CHECK(ctx.cfg.pan_id == 0x1234);
    CHECK(ctx.cfg.have_pan_id == true);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main() {
    /* existing */
    test_ping();
    test_version();
    test_extaddr();
    test_unknown();
    test_reset_req();
    test_reset_ind();
    /* new */
    test_netcfg();
    test_nv_write_dispatch();

    if (g_fail) { printf("%d CHECK(s) failed\n", g_fail); return 1; }
    printf("all znp_dispatch tests passed\n");
    return 0;
}
