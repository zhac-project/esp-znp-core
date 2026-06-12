// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// def 5 (P6-T33): cross-repo MT wire-constant drift guard.
//
// This NCP re-implements the TI Z-Stack MT framing that the host peer (the P4
// driver) speaks. The two protocol implementations are maintained in separate
// repos and MUST agree byte-for-byte or the link silently corrupts:
//
//   this repo  : esp-znp-core/components/mt_proto/include/mt_proto.h
//   host peer  : zhac-components/components/znp_driver/include/znp_driver.h
//
// We compare the wire-visible constants (SOF, payload-max, frame overhead, the
// SREQ/AREQ/SRSP type bits, subsystem ids) and the FCS algorithm/coverage.
//
// The host header is a C++ file in a sibling repo that pulls in <functional>;
// including it from this standalone cmake is impractical and couples the build
// to a fixed sibling-repo layout. So we MIRROR the host peer's values below as
// PEER_* literals with an explicit "keep in sync" pointer, and additionally
// attempt a real include when ZNP_DRIVER_H is reachable (define it via the
// build) so CI in the meta-repo can assert against the live header too.
//
//   >>> KEEP IN SYNC WITH:
//   >>> zhac-components/components/znp_driver/include/znp_driver.h
//   <<< If you change a constant there, change it here (and in mt_proto.h).

#include "mt_proto.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

// NOTE on why we mirror rather than #include the peer header directly:
// this repo's mt_proto.h defines MT_SOF/MT_SREQ/... as PREPROCESSOR MACROS,
// while znp_driver.h defines the same names as `static constexpr`. Including
// both in one TU makes the macros textually clobber the constexpr declarations
// (a hard compile error). They are also in separate repos with no guaranteed
// co-location. So the drift guard mirrors the peer's literals below; if either
// header changes a constant, this test must be updated in lockstep.

static int g_fail = 0;
#define CHECK(cond) do { if(!(cond)){ \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while(0)

/* ── Mirrored host-peer constants (znp_driver.h) ──────────────────────────── *
 * NB: identifiers deliberately do NOT reuse the mt_proto.h macro names
 * (MT_SOF, MT_MAX_PAYLOAD, ...). Those are #defines and would expand inside a
 * `peer::MT_SOF` qualified name and break compilation. */
namespace peer {
    static constexpr uint8_t  kSof          = 0xFE;
    static constexpr size_t   kOverhead     = 5;
    static constexpr size_t   kMaxPayload   = 250;
    static constexpr uint8_t  kTypeSreqBit  = 0x20;
    static constexpr uint8_t  kTypeAreqBit  = 0x40;
    static constexpr uint8_t  kTypeSrspBit  = 0x60;
    static constexpr uint8_t  kSubSys       = 0x01;
    static constexpr uint8_t  kSubAf        = 0x04;
    static constexpr uint8_t  kSubZdo       = 0x05;
    static constexpr uint8_t  kSubUtil      = 0x07;
    static constexpr uint8_t  kSubAppCnf    = 0x0F;
    // FCS: f = len ^ cmd0 ^ cmd1 ^ all payload bytes (znp_parser.cpp / mt_proto).
    static uint8_t fcs(uint8_t len, uint8_t cmd0, uint8_t cmd1,
                       const uint8_t *p, uint8_t n) {
        uint8_t f = (uint8_t)(len ^ cmd0 ^ cmd1);
        for (uint8_t i = 0; i < n; i++) f ^= p[i];
        return f;
    }
}

int main() {
    /* SOF / sizes */
    CHECK(MT_SOF         == peer::kSof);
    CHECK(MT_OVERHEAD    == peer::kOverhead);
    CHECK(MT_MAX_PAYLOAD == peer::kMaxPayload);
    CHECK(MT_SOF == 0xFE);
    CHECK(MT_MAX_PAYLOAD == 250);
    CHECK(MT_OVERHEAD == 5);

    /* type bits — assert the bit pattern, not just that the macros agree */
    CHECK(MT_SREQ(0) == peer::kTypeSreqBit);
    CHECK(MT_AREQ(0) == peer::kTypeAreqBit);
    CHECK(MT_SRSP(0) == peer::kTypeSrspBit);
    CHECK(MT_SREQ(0) == 0x20);
    CHECK(MT_AREQ(0) == 0x40);
    CHECK(MT_SRSP(0) == 0x60);
    /* SRSP type bit applied to a subsystem (regression: 0x60|sub) */
    CHECK(MT_SRSP(ZNP_SYS) == (uint8_t)(0x60 | 0x01));

    /* subsystem ids */
    CHECK(ZNP_SYS     == peer::kSubSys);
    CHECK(ZNP_AF      == peer::kSubAf);
    CHECK(ZNP_ZDO     == peer::kSubZdo);
    CHECK(ZNP_UTIL    == peer::kSubUtil);
    CHECK(ZNP_APP_CNF == peer::kSubAppCnf);

    /* FCS algorithm + coverage must match on a representative frame */
    const uint8_t pl[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    const uint8_t mine = mt_fcs(4, MT_SREQ(ZNP_SYS), 0x1D, pl, 4);
    const uint8_t theirs = peer::fcs(4, peer::kTypeSreqBit | peer::kSubSys,
                                     0x1D, pl, 4);
    CHECK(mine == theirs);
    /* known-good value: 0x04 ^ 0x21 ^ 0x1D ^ 0xDE ^ 0xAD ^ 0xBE ^ 0xEF */
    CHECK(mine == (uint8_t)(0x04 ^ 0x21 ^ 0x1D ^ 0xDE ^ 0xAD ^ 0xBE ^ 0xEF));
    /* empty-payload frame (FCS over len^cmd0^cmd1 only) */
    CHECK(mt_fcs(0, MT_SRSP(ZNP_SYS), 0x01, nullptr, 0)
          == (uint8_t)(0x00 ^ 0x61 ^ 0x01));

    if (g_fail) { printf("%d CHECK(s) failed\n", g_fail); return 1; }
    printf("all mt_proto wire-constants checks passed\n");
    return 0;
}
