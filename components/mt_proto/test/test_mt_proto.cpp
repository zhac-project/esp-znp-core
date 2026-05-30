// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
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

    /* non-empty payload encodes correctly (full encode path, not just mt_fcs) */
    const uint8_t pl[2] = {0x79, 0x00};
    mt_frame_t srsp = { MT_SRSP(ZNP_SYS), 0x01, 2, pl };   /* 0x61,0x01 */
    uint8_t b2[16];
    size_t n2 = mt_encode(&srsp, b2, sizeof(b2));
    CHECK(n2 == 7);
    const uint8_t expect2[7] = {0xFE, 0x02, 0x61, 0x01, 0x79, 0x00, 0x1B};
    CHECK(memcmp(b2, expect2, 7) == 0);
    CHECK(b2[6] == mt_fcs(2, 0x61, 0x01, pl, 2));

    /* exact-fit buffer (buf_size == total) succeeds */
    uint8_t exact[5];
    CHECK(mt_encode(&ping, exact, sizeof(exact)) == 5);

    /* payload over MT_MAX_PAYLOAD (251 > 250) is rejected */
    static uint8_t big[251] = {0};
    mt_frame_t toobig = { MT_SREQ(ZNP_AF), 0x01, 251, big };
    uint8_t b3[260];
    CHECK(mt_encode(&toobig, b3, sizeof(b3)) == 0);
}

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

    /* payload length exceeds MT_MAX_PAYLOAD -> overflow */
    const uint8_t overflow[5] = {0xFE, 0xFF, 0x21, 0x01, 0x00};
    CHECK(mt_decode(overflow, 5, &out) == MT_DECODE_OVERFLOW);

    /* header present but buffer short of declared payload -> truncated (second guard) */
    const uint8_t shortpl[6] = {0xFE, 0x02, 0x61, 0x01, 0x79, 0x00};  /* len=6, needs 7 */
    CHECK(mt_decode(shortpl, 6, &out) == MT_DECODE_TRUNCATED);
}

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

    /* recovery: after the bad-FCS reset above, a subsequent valid frame still parses */
    CHECK(mt_parser_feed(&p, ping[0], &out) == 0);
    CHECK(mt_parser_feed(&p, ping[1], &out) == 0);
    CHECK(mt_parser_feed(&p, ping[2], &out) == 0);
    CHECK(mt_parser_feed(&p, ping[3], &out) == 0);
    CHECK(mt_parser_feed(&p, ping[4], &out) == 1);
    CHECK(out.cmd0 == 0x21 && out.cmd1 == 0x01);

    /* two consecutive valid frames reusing the same parser instance */
    const uint8_t srsp2[7] = {0xFE, 0x02, 0x61, 0x01, 0x79, 0x00, 0x1B};
    int d2 = 0;
    for (int i = 0; i < 7; i++) d2 = mt_parser_feed(&p, srsp2[i], &out);
    CHECK(d2 == 1 && out.payload_len == 2 && out.payload[0] == 0x79);
    int d3 = 0;
    for (int i = 0; i < 7; i++) d3 = mt_parser_feed(&p, srsp2[i], &out);
    CHECK(d3 == 1 && out.payload_len == 2 && out.payload[1] == 0x00);
}

int main() {
    test_fcs();
    test_encode();
    test_decode();
    test_parser();
    if (g_fail) { printf("%d CHECK(s) failed\n", g_fail); return 1; }
    printf("all mt_proto tests passed\n");
    return 0;
}
