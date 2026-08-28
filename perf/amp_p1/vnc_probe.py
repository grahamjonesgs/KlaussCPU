#!/usr/bin/env python3
"""Minimal RFB 3.8 probe for the KlaussCPU VNC server (core 1 Zephyr or
AMP core 2).  Keeps the server's native RGB565 LE format, offers Hextile+Raw
or Raw-only, then drives FramebufferUpdateRequests for N seconds measuring
updates/sec, bytes/update and the encodings the server chose.

usage: vnc_probe.py HOST [--raw-only] [--full] [--seconds N] [--label TEXT]
  --full: non-incremental requests every time (full-frame service rate)"""
import socket, struct, sys, time

HT_RAW, HT_BG, HT_FG, HT_SUB = 1, 2, 4, 8


def rx(s, n):
    b = b""
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise ConnectionError("server closed")
        b += c
    return b


def handshake(s, hextile):
    assert rx(s, 12).startswith(b"RFB ")
    s.sendall(b"RFB 003.008\n")
    types = rx(s, rx(s, 1)[0])
    assert 1 in types
    s.sendall(bytes([1]))
    assert struct.unpack(">I", rx(s, 4))[0] == 0
    s.sendall(bytes([1]))
    si = rx(s, 24)
    w, h = struct.unpack(">HH", si[:4])
    name = rx(s, struct.unpack(">I", si[20:24])[0]).decode(errors="replace")
    encs = [5, 0] if hextile else [0]
    s.sendall(struct.pack(">BxH", 2, len(encs)) + b"".join(struct.pack(">i", e) for e in encs))
    return w, h, name


def req(s, w, h, incr):
    s.sendall(struct.pack(">BBHHHH", 3, 1 if incr else 0, 0, 0, w, h))


def hextile_rect(s, rw, rh, bpp):
    n = 0
    for ty in range(0, rh, 16):
        th = min(16, rh - ty)
        for tx in range(0, rw, 16):
            tw = min(16, rw - tx)
            sub = rx(s, 1)[0]; n += 1
            if sub & HT_RAW:
                n += len(rx(s, tw * th * bpp)); continue
            if sub & HT_BG: n += len(rx(s, bpp))
            if sub & HT_FG: n += len(rx(s, bpp))
            if sub & HT_SUB:
                cnt = rx(s, 1)[0]; n += 1 + len(rx(s, 2 * cnt))
    return n


def read_update(s, bpp):
    hdr = rx(s, 4)
    assert hdr[0] == 0, hdr
    total, encs = 4, {}
    for _ in range(struct.unpack(">H", hdr[2:4])[0]):
        x, y, rw, rh, enc = struct.unpack(">HHHHi", rx(s, 12)); total += 12
        encs[enc] = encs.get(enc, 0) + 1
        if enc == 0: total += len(rx(s, rw * rh * bpp))
        elif enc == 5: total += hextile_rect(s, rw, rh, bpp)
        else: raise AssertionError(enc)
    return total, encs


def main():
    host = sys.argv[1]; raw = "--raw-only" in sys.argv; full = "--full" in sys.argv
    secs = int(sys.argv[sys.argv.index("--seconds") + 1]) if "--seconds" in sys.argv else 30
    label = sys.argv[sys.argv.index("--label") + 1] if "--label" in sys.argv else "probe"
    s = socket.create_connection((host, 5900), timeout=30); s.settimeout(30)
    w, h, name = handshake(s, not raw)
    print(f"[{label}] connected: '{name}' {w}x{h}, encodings={'raw-only' if raw else 'hextile+raw'}")
    req(s, w, h, False); n, e = read_update(s, 2)
    print(f"[{label}] first full update: {n} B, encodings {e}")
    t0 = time.time(); ups = 0; tot = 0; ec = {}
    while time.time() - t0 < secs:
        req(s, w, h, not full); n, e = read_update(s, 2)
        ups += 1; tot += n
        for k, v in e.items(): ec[k] = ec.get(k, 0) + v
    dt = time.time() - t0
    print(f"[{label}] {ups} updates in {dt:.1f}s = {ups/dt:.2f} fps, avg {tot//max(ups,1)} B/update, "
          f"{tot/dt/1024:.0f} KB/s, rect encodings {ec}")


if __name__ == "__main__":
    main()
