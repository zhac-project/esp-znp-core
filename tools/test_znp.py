#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
# SPDX-License-Identifier: AGPL-3.0-or-later
"""
esp-znp-core smoke + protocol test
==================================

Two modes — pick based on what's wired:

  --console PORT
      Read the H2's USB-Serial-JTAG console (e.g. /dev/ttyACM0) and
      scan the boot log for known-good lines. Sanity check that the
      firmware is alive. Does NOT exercise the MT wire.

      Boot logs only emit once at startup, so reboot the chip
      (RESET button) during the read window — or pass --reset to
      DTR-toggle (works if the dev kit routes it).

  --mt PORT
      Speak the TI-MT protocol over UART1 (H2 defaults: TX = GPIO5,
      RX = GPIO4). Needs a USB-UART adapter wired 3.3 V / common GND
      to those pins. Sends SYS_PING / SYS_VERSION / SYS_GET_EXTADDR,
      validates the SRSPs byte-by-byte, listens for SYS_RESET_IND.

Usage:
    pip install pyserial   # one-off
    ./tools/test_znp.py --console /dev/ttyACM0
    ./tools/test_znp.py --mt /dev/ttyUSB0
    ./tools/test_znp.py --mt /dev/ttyUSB0 --reset

Exit codes: 0 = pass, 1 = test failure, 2 = setup error.
"""
import argparse, re, sys, time

try:
    import serial
except ImportError:
    print("ERR: pyserial not installed.  pip install pyserial",
          file=sys.stderr)
    sys.exit(2)


# ─── MT codec (mirrors mt_proto in firmware) ─────────────────────────
SOF          = 0xFE
MT_SREQ_SYS  = 0x21    # 0x20 | SYS(0x01)
MT_SRSP_SYS  = 0x61    # 0x60 | SYS(0x01)
MT_AREQ_SYS  = 0x41    # 0x40 | SYS(0x01)
SYS_RESET_IND_CMD1 = 0x80


def fcs(data: bytes) -> int:
    """XOR of every byte (matches mt_fcs in firmware)."""
    x = 0
    for b in data:
        x ^= b
    return x


def encode(cmd0: int, cmd1: int, payload: bytes = b"") -> bytes:
    if len(payload) > 250:
        raise ValueError("payload > 250 bytes")
    hdr = bytes([len(payload), cmd0, cmd1])
    return bytes([SOF]) + hdr + payload + bytes([fcs(hdr + payload)])


class Parser:
    """Streaming MT frame reassembler. feed(bytes) → list of (cmd0,cmd1,payload)."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, b: bytes):
        self.buf += b
        out = []
        while True:
            try:
                i = self.buf.index(SOF)
            except ValueError:
                self.buf.clear()
                break
            if i:
                del self.buf[:i]    # drop pre-SOF garbage
            if len(self.buf) < 5:
                break
            plen = self.buf[1]
            need = 5 + plen
            if plen > 250:
                # impossible — drop the bad SOF and rescan
                del self.buf[0]
                continue
            if len(self.buf) < need:
                break
            frame = bytes(self.buf[:need])
            del self.buf[:need]
            exp = fcs(frame[1:4 + plen])
            if exp != frame[4 + plen]:
                # bad FCS — rescan starting from the byte after SOF
                self.buf = bytearray(frame[1:]) + self.buf
                continue
            out.append((frame[2], frame[3], frame[4:4 + plen]))
        return out


# ─── console mode ────────────────────────────────────────────────────
EXPECTED_BOOT_PATTERNS = [
    # IDF identity — confirms *this* firmware is what's on the chip.
    re.compile(r"app_init: Project name:\s+esp-znp-core"),
    # znp_uart spun up on the configured port + baud.
    re.compile(r"znp_uart: UART\d+ up @ 115200 8N1"),
    # app_main reached its final banner — UART wired + boot RESET_IND sent.
    re.compile(r"esp-znp-core MT-NCP up: SYS link ready, RESET_IND sent"),
]


def test_console(port: str, timeout_s: float, do_reset: bool) -> int:
    print(f"==> console smoke test on {port} (window {timeout_s:.1f}s)")
    try:
        ser = serial.Serial(port, 115200, timeout=0.2)
    except Exception as e:
        print(f"ERR: cannot open {port}: {e}", file=sys.stderr)
        return 2

    if do_reset:
        print("  · pulsing DTR/RTS for reset (dev kit may ignore)")
        ser.dtr = False; ser.rts = False; time.sleep(0.05)
        ser.dtr = True;  ser.rts = True

    end = time.time() + timeout_s
    seen = [False] * len(EXPECTED_BOOT_PATTERNS)
    captured = []
    while time.time() < end and not all(seen):
        line = ser.readline()
        if not line:
            continue
        s = line.decode("utf-8", errors="replace").rstrip("\r\n")
        if s:
            captured.append(s)
        for i, rx in enumerate(EXPECTED_BOOT_PATTERNS):
            if not seen[i] and rx.search(s):
                seen[i] = True
                print(f"  ✓ {rx.pattern}")
    ser.close()

    missing = [rx.pattern for rx, s in zip(EXPECTED_BOOT_PATTERNS, seen) if not s]
    if missing:
        print(f"FAIL: missing expected log lines:", file=sys.stderr)
        for p in missing:
            print(f"      {p}", file=sys.stderr)
        if captured:
            print("Last 25 lines captured:", file=sys.stderr)
            for ln in captured[-25:]:
                print(f"  | {ln}", file=sys.stderr)
        else:
            print("(no log lines captured — chip not booting, or already booted "
                  "past the banner. Press RESET on the dev kit during the window, "
                  "or rerun with --reset.)", file=sys.stderr)
        return 1
    print("PASS: console boot smoke")
    return 0


# ─── MT mode ─────────────────────────────────────────────────────────
def test_mt(port: str, force_reset: bool, timeout_s: float) -> int:
    print(f"==> MT protocol test on {port}")
    try:
        ser = serial.Serial(port, 115200, bytesize=8, parity="N",
                            stopbits=1, rtscts=False, timeout=0.2)
    except Exception as e:
        print(f"ERR: cannot open {port}: {e}", file=sys.stderr)
        return 2

    parser = Parser()
    fails = 0

    if force_reset:
        print("  · DTR-toggling to nudge a reset")
        ser.dtr = False; time.sleep(0.05); ser.dtr = True

    # Capture any pending RESET_IND for ~1.5 s, then move on.
    time.sleep(0.2)
    end = time.time() + 1.5
    reset_payload = None
    while time.time() < end:
        n = ser.read(64)
        if not n:
            continue
        for c0, c1, pl in parser.feed(n):
            if c0 == MT_AREQ_SYS and c1 == SYS_RESET_IND_CMD1:
                reset_payload = pl
                break
        if reset_payload is not None:
            break
    if reset_payload is not None:
        print(f"  ✓ SYS_RESET_IND payload = {reset_payload.hex()}")
    else:
        print("  · no SYS_RESET_IND seen (chip likely booted earlier — fine)")

    def srsp_for(cmd1: int, payload: bytes = b"") -> bytes | None:
        frame = encode(MT_SREQ_SYS, cmd1, payload)
        ser.reset_input_buffer()
        parser.buf.clear()
        ser.write(frame)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            n = ser.read(64)
            if not n:
                continue
            for c0, c1, pl in parser.feed(n):
                if c0 == MT_SRSP_SYS and c1 == cmd1:
                    return pl
        return None

    # SYS_PING (cmd1 0x01) → 2-byte capability bitmap = 0x0179 → {0x79, 0x01}
    pl = srsp_for(0x01)
    if pl is None:
        print("FAIL: SYS_PING — no SRSP", file=sys.stderr); fails += 1
    elif pl != b"\x79\x01":
        print(f"FAIL: SYS_PING SRSP {pl.hex()} != 7901", file=sys.stderr); fails += 1
    else:
        print(f"  ✓ SYS_PING        capabilities = 0x{pl[0] | (pl[1] << 8):04x}")

    # SYS_VERSION (cmd1 0x02) → 5-byte {TransportRev, Product, Major, Minor, Maint}
    pl = srsp_for(0x02)
    expect_ver = b"\x02\x00\x02\x07\x01"
    if pl is None:
        print("FAIL: SYS_VERSION — no SRSP", file=sys.stderr); fails += 1
    elif pl != expect_ver:
        print(f"FAIL: SYS_VERSION SRSP {pl.hex()} != {expect_ver.hex()}",
              file=sys.stderr); fails += 1
    else:
        tr, pr, mj, mn, mt = pl
        print(f"  ✓ SYS_VERSION     transport={tr} product={pr} v{mj}.{mn}.{mt}")

    # SYS_GET_EXTADDR (cmd1 0x04) → 8-byte IEEE EUI-64 (LE on wire)
    pl = srsp_for(0x04)
    if pl is None:
        print("FAIL: SYS_GET_EXTADDR — no SRSP", file=sys.stderr); fails += 1
    elif len(pl) != 8:
        print(f"FAIL: SYS_GET_EXTADDR SRSP length {len(pl)} != 8",
              file=sys.stderr); fails += 1
    else:
        ieee_be = pl[::-1]   # display MSB-first
        print(f"  ✓ SYS_GET_EXTADDR IEEE = {':'.join(f'{b:02x}' for b in ieee_be)}")
        if all(b == 0 for b in pl):
            print("    (IEEE is all-zero — esp_read_mac failed or efuse blank; "
                  "P4 tolerates this as a bind fallback.)")

    ser.close()
    if fails:
        print(f"FAIL: {fails} MT subtest(s) failed", file=sys.stderr)
        return 1
    print("PASS: MT SYS subset")
    return 0


# ─── entry ───────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--console", metavar="PORT",
                   help="USB-Serial-JTAG console (e.g. /dev/ttyACM0)")
    g.add_argument("--mt", metavar="PORT",
                   help="USB-UART on H2 UART1 (GPIO5 TX / GPIO4 RX)")
    ap.add_argument("--reset", action="store_true",
                    help="toggle DTR/RTS for reset before reading")
    ap.add_argument("--timeout", type=float, default=8.0,
                    help="console-mode read window seconds, "
                         "or MT-mode per-SRSP timeout (default 8.0)")
    args = ap.parse_args()
    if args.console:
        sys.exit(test_console(args.console, args.timeout, args.reset))
    else:
        sys.exit(test_mt(args.mt, args.reset, args.timeout))


if __name__ == "__main__":
    main()
