#include "znp_dispatch.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while(0)

/* ── fake backend ────────────────────────────────────────────────────────── */
static uint8_t  s_ieee[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
static int      s_reset_calls       = 0;
static int      s_apply_cfg_calls   = 0;
static int      s_start_calls       = 0;
static int      s_bdb_commission_calls = 0;
static uint8_t  s_last_bdb_mode     = 0xFF;
/* call-order log: 0=apply_config, 1=start_stack (max 2 entries per test) */
static int      s_call_log[2];
static int      s_call_log_idx      = 0;

static void fake_get_ieee(uint8_t out[8]) { memcpy(out, s_ieee, 8); }
static void fake_reset(void)              { s_reset_calls++; }
static void fake_apply_config(const znp_netcfg_t *cfg) {
    (void)cfg;
    if (s_call_log_idx < 2) s_call_log[s_call_log_idx++] = 0;
    s_apply_cfg_calls++;
}
static bool fake_start_stack(void) {
    if (s_call_log_idx < 2) s_call_log[s_call_log_idx++] = 1;
    s_start_calls++;
    return true;
}
static bool fake_bdb_commission(uint8_t ezb_mode) {
    s_last_bdb_mode = ezb_mode;
    s_bdb_commission_calls++;
    return true;
}

static const znp_backend_t FAKE = {
    .get_ieee       = fake_get_ieee,
    .request_reset  = fake_reset,
    .apply_config   = fake_apply_config,
    .start_stack    = fake_start_stack,
    .bdb_commission = fake_bdb_commission,
    .get_nwk_info   = NULL,
    .permit_join    = NULL,
};

static void reset_fake_state(void) {
    s_reset_calls        = 0;
    s_apply_cfg_calls    = 0;
    s_start_calls        = 0;
    s_bdb_commission_calls = 0;
    s_last_bdb_mode      = 0xFF;
    s_call_log[0]        = -1;
    s_call_log[1]        = -1;
    s_call_log_idx       = 0;
}

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

/* ── Task 4.2: ZDO_STARTUP_FROM_APP ──────────────────────────────────────── */

static void test_startup_from_app(void) {
    reset_fake_state();
    /* Build ZDO_STARTUP_FROM_APP: MT_SREQ(ZNP_ZDO)/0x40, payload={0x00,0x00} */
    const uint8_t payload[2] = {0x00, 0x00};
    mt_frame_t r = { MT_SREQ(ZNP_ZDO), 0x40, sizeof(payload), payload };
    uint8_t buf[32];
    znp_dispatch_ctx ctx = make_ctx();

    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));

    /* SRSP: FE + len(1) + MT_SRSP(ZNP_ZDO)(1) + 0x40(1) + status(1) + fcs(1) = 6 bytes */
    CHECK(n == 6);
    CHECK(buf[0] == 0xFE);
    CHECK(buf[1] == 0x01);                 /* payload len = 1 (status byte) */
    CHECK(buf[2] == MT_SRSP(ZNP_ZDO));     /* 0x65 */
    CHECK(buf[3] == 0x40);
    CHECK(buf[4] == 0x00);                 /* status = success */

    /* apply_config called before start_stack */
    CHECK(s_apply_cfg_calls == 1);
    CHECK(s_start_calls == 1);
    CHECK(s_call_log[0] == 0);   /* apply_config first */
    CHECK(s_call_log[1] == 1);   /* start_stack second */
}

/* ── Task 4.2: APP_CNF BDB_START_COMMISSIONING ────────────────────────────── */

static void test_bdb_start_formation(void) {
    reset_fake_state();
    /* TI mode 0x04 = FORMATION → EZB 0x08 */
    const uint8_t payload[1] = {0x04};
    mt_frame_t r = { MT_SREQ(ZNP_APP_CNF), 0x05, 1, payload };
    uint8_t buf[32];
    znp_dispatch_ctx ctx = make_ctx();

    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));

    /* SRSP: MT_SRSP(ZNP_APP_CNF)/0x05, status 0x00 */
    CHECK(n == 6);
    CHECK(buf[2] == MT_SRSP(ZNP_APP_CNF));
    CHECK(buf[3] == 0x05);
    CHECK(buf[4] == 0x00);

    CHECK(s_bdb_commission_calls == 1);
    CHECK(s_last_bdb_mode == ZNP_EZB_BDB_FORMATION);   /* 0x08 */
}

static void test_bdb_start_steering(void) {
    reset_fake_state();
    /* TI mode 0x02 = STEERING → EZB 0x04 */
    const uint8_t payload[1] = {0x02};
    mt_frame_t r = { MT_SREQ(ZNP_APP_CNF), 0x05, 1, payload };
    uint8_t buf[32];
    znp_dispatch_ctx ctx = make_ctx();

    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));

    CHECK(n == 6);
    CHECK(buf[2] == MT_SRSP(ZNP_APP_CNF));
    CHECK(buf[3] == 0x05);
    CHECK(buf[4] == 0x00);

    CHECK(s_bdb_commission_calls == 1);
    CHECK(s_last_bdb_mode == ZNP_EZB_BDB_STEERING);    /* 0x04 */
}

/* ── Task 4.2: APP_CNF BDB_SET_CHANNEL ───────────────────────────────────── */

static void test_bdb_set_channel(void) {
    reset_fake_state();
    /* isPrimary=1, chan_mask ch11=0x00000800 */
    const uint8_t payload[5] = {0x01, 0x00, 0x08, 0x00, 0x00};
    mt_frame_t r = { MT_SREQ(ZNP_APP_CNF), 0x08, 5, payload };
    uint8_t buf[32];
    znp_dispatch_ctx ctx = make_ctx();

    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));

    /* SRSP: MT_SRSP(ZNP_APP_CNF)/0x08, status 0x00 */
    CHECK(n == 6);
    CHECK(buf[2] == MT_SRSP(ZNP_APP_CNF));
    CHECK(buf[3] == 0x08);
    CHECK(buf[4] == 0x00);
}

/* ── Task 4.2 (robustness): start_stack returns false → SRSP status 0x01 ─── */

static bool fail_start_stack(void) { return false; }
static bool fail_bdb_commission(uint8_t /*ezb_mode*/) { return false; }

static const znp_backend_t FAIL_BE = {
    .get_ieee       = nullptr,
    .request_reset  = nullptr,
    .apply_config   = nullptr,
    .start_stack    = fail_start_stack,
    .bdb_commission = fail_bdb_commission,
    .get_nwk_info   = nullptr,
    .permit_join    = nullptr,
};

static void test_startup_failure(void) {
    /* start_stack returns false → STARTUP_FROM_APP SRSP status must be 0x01 */
    znp_dispatch_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.be = &FAIL_BE;

    const uint8_t payload[2] = {0x00, 0x00};
    mt_frame_t r = { MT_SREQ(ZNP_ZDO), 0x40, sizeof(payload), payload };
    uint8_t buf[32];

    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));
    CHECK(n == 6);
    CHECK(buf[2] == MT_SRSP(ZNP_ZDO));
    CHECK(buf[3] == 0x40);
    CHECK(buf[4] == 0x01);   /* failure status */
}

/* ── Task 4.3: ZDO_EXT_NWK_INFO (0x25/0x50) ──────────────────────────────── */

static int      s_get_nwk_info_calls = 0;
static bool     s_nwk_info_return    = true;
static uint16_t s_nwk_info_panid     = 0x1234;
static uint16_t s_nwk_info_short     = 0x0000;
static uint8_t  s_nwk_info_devstate  = 0x09;

static bool fake_get_nwk_info(uint16_t *panid, uint16_t *short_addr, uint8_t *dev_state) {
    s_get_nwk_info_calls++;
    *panid      = s_nwk_info_panid;
    *short_addr = s_nwk_info_short;
    *dev_state  = s_nwk_info_devstate;
    return s_nwk_info_return;
}

/* FAKE_NWK: like FAKE but with get_nwk_info wired */
static const znp_backend_t FAKE_NWK = {
    .get_ieee       = fake_get_ieee,
    .request_reset  = fake_reset,
    .apply_config   = fake_apply_config,
    .start_stack    = fake_start_stack,
    .bdb_commission = fake_bdb_commission,
    .get_nwk_info   = fake_get_nwk_info,
    .permit_join    = NULL,
};

static void test_ext_nwk_info(void) {
    reset_fake_state();
    s_get_nwk_info_calls = 0;
    s_nwk_info_panid     = 0x1234;
    s_nwk_info_short     = 0x0000;
    s_nwk_info_devstate  = 0x09;

    /* ZDO_EXT_NWK_INFO: MT_SREQ(ZNP_ZDO)=0x25, cmd1=0x50, no payload */
    mt_frame_t r = { MT_SREQ(ZNP_ZDO), 0x50, 0, nullptr };
    uint8_t buf[32];
    znp_dispatch_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.be = &FAKE_NWK;

    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));

    /* SRSP: FE + len(1) + MT_SRSP(ZNP_ZDO)(1) + 0x50(1) + payload(7) + fcs(1) = 12 bytes
     * P4 reads (zigbee_mgr.cpp:427-429):
     *   payload[0..1] = shortaddr LE
     *   payload[2]    = devstate
     *   payload[3..4] = panid LE
     *   payload[5..6] = zeros (remaining fields not used by P4)
     * Expected bytes: FE 07 65 50 00 00 09 34 12 00 00 1D */
    const uint8_t expect[12] = {0xFE,0x07,0x65,0x50,
                                 0x00,0x00,  /* shortaddr LE = 0x0000 */
                                 0x09,       /* devstate = DEV_ZB_COORD */
                                 0x34,0x12,  /* panid LE = 0x1234 */
                                 0x00,0x00,  /* padding to reach >= 7 bytes */
                                 0x1D};      /* FCS */
    CHECK(n == 12);
    CHECK(memcmp(buf, expect, 12) == 0);
    CHECK(s_get_nwk_info_calls == 1);
    /* Verify individual offsets explicitly (mirrors zigbee_mgr.cpp parse) */
    CHECK(buf[4] == 0x00 && buf[5] == 0x00);   /* shortaddr */
    CHECK(buf[6] == 0x09);                      /* devstate */
    CHECK(buf[7] == 0x34 && buf[8] == 0x12);   /* panid */
}

static void test_ext_nwk_info_null_backend(void) {
    /* NULL get_nwk_info → still returns SRSP with zeros, no crash */
    static const znp_backend_t NULL_NWK_BE = {
        .get_ieee       = nullptr,
        .request_reset  = nullptr,
        .apply_config   = nullptr,
        .start_stack    = nullptr,
        .bdb_commission = nullptr,
        .get_nwk_info   = nullptr,
        .permit_join    = nullptr,
    };
    znp_dispatch_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.be = &NULL_NWK_BE;

    mt_frame_t r = { MT_SREQ(ZNP_ZDO), 0x50, 0, nullptr };
    uint8_t buf[32];
    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));

    /* Must return a valid SRSP (not 0), all payload bytes zero */
    CHECK(n == 12);
    CHECK(buf[0] == 0xFE);
    CHECK(buf[2] == MT_SRSP(ZNP_ZDO));
    CHECK(buf[3] == 0x50);
    /* shortaddr=0, devstate=0, panid=0 */
    CHECK(buf[4] == 0x00 && buf[5] == 0x00);
    CHECK(buf[6] == 0x00);
    CHECK(buf[7] == 0x00 && buf[8] == 0x00);
}

/* ── Task 4.3: AF_REGISTER (0x24/0x00) ──────────────────────────────────── */

static void test_af_register(void) {
    /* AF_REGISTER: MT_SREQ(ZNP_AF)=0x24, cmd1=0x00
     * P4 sends: endpoint(1) + profile(2) + device_id(2) + version(1) +
     *           latency(1) + in_count(1) + out_count(1) = 9 bytes min
     * zigbee_mgr.cpp:895: cmd0=MT_SREQ(ZNP_AF), cmd1=0x00
     * SRSP: MT_SRSP(ZNP_AF)/0x00, 1-byte status=0x00
     * Expected bytes: FE 01 64 00 00 65 */
    const uint8_t payload[9] = {
        0x01,        /* endpoint = 1 */
        0x04, 0x01,  /* app_profile_id = 0x0104 HA */
        0x05, 0x00,  /* app_device_id = 0x0005 */
        0x00,        /* device version */
        0x00,        /* latency */
        0x00,        /* in_cluster_count = 0 */
        0x00,        /* out_cluster_count = 0 */
    };
    mt_frame_t r = { MT_SREQ(ZNP_AF), 0x00, sizeof(payload), payload };
    uint8_t buf[32];
    znp_dispatch_ctx ctx = make_ctx();

    size_t n = znp_dispatch(&r, &ctx, buf, sizeof(buf));

    /* SRSP: FE + len(1) + MT_SRSP(ZNP_AF)(1) + 0x00(1) + status(1) + fcs(1) = 6 bytes
     * Expected bytes: FE 01 64 00 00 65 */
    const uint8_t expect[6] = {0xFE, 0x01, 0x64, 0x00, 0x00, 0x65};
    CHECK(n == 6);
    CHECK(memcmp(buf, expect, 6) == 0);
    CHECK(buf[2] == MT_SRSP(ZNP_AF));   /* 0x64 */
    CHECK(buf[3] == 0x00);
    CHECK(buf[4] == 0x00);              /* status = success */
}

/* ── Task 4.2: null-guard — NULL backend members don't crash ─────────────── */

static void test_startup_null_backend(void) {
    /* Backend with all new ops NULL — must not crash, must still SRSP */
    static const znp_backend_t NULL_BE = {
        .get_ieee       = nullptr,
        .request_reset  = nullptr,
        .apply_config   = nullptr,
        .start_stack    = nullptr,
        .bdb_commission = nullptr,
        .get_nwk_info   = nullptr,
        .permit_join    = nullptr,
    };
    znp_dispatch_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.be = &NULL_BE;

    uint8_t buf[32];

    /* ZDO_STARTUP_FROM_APP with NULL ops */
    const uint8_t pl2[2] = {0x00, 0x00};
    mt_frame_t r1 = { MT_SREQ(ZNP_ZDO), 0x40, 2, pl2 };
    size_t n1 = znp_dispatch(&r1, &ctx, buf, sizeof(buf));
    CHECK(n1 == 6);
    CHECK(buf[4] == 0x00);

    /* BDB_START_COMMISSIONING with NULL bdb_commission */
    const uint8_t pl1[1] = {0x04};
    mt_frame_t r2 = { MT_SREQ(ZNP_APP_CNF), 0x05, 1, pl1 };
    size_t n2 = znp_dispatch(&r2, &ctx, buf, sizeof(buf));
    CHECK(n2 == 6);
    CHECK(buf[4] == 0x00);
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
    /* new (task 4.1) */
    test_netcfg();
    test_nv_write_dispatch();
    /* new (task 4.2) */
    test_startup_from_app();
    test_bdb_start_formation();
    test_bdb_start_steering();
    test_bdb_set_channel();
    test_startup_null_backend();
    test_startup_failure();
    /* new (task 4.3) */
    test_ext_nwk_info();
    test_ext_nwk_info_null_backend();
    test_af_register();

    if (g_fail) { printf("%d CHECK(s) failed\n", g_fail); return 1; }
    printf("all znp_dispatch tests passed\n");
    return 0;
}
