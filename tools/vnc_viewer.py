#!/usr/bin/env python3
"""vnc_viewer.py — minimal VNC viewer for the KlaussCPU VNC servers.

Mainstream viewers (TigerVNC, RealVNC, UltraVNC) only offer full colour
(32 bpp) or 8 bpp-and-below, so against the board they force a per-pixel
convert and twice the bytes.  This viewer asks for the framebuffer's native
format — RGB565 little-endian — so the server takes its no-convert fast path
(core 1 Zephyr vnc_server.c or AMP core 2 vnc_c2.c alike).  Against AMP core 2
("KlaussCPU core2") it defaults to 8-bit colour-map pixels instead: doom's own
palette indices, half the bytes again, and core 1 skips its RGB565 pass.

Standard library only (tkinter).  The title bar shows received updates/s,
displayed frames/s and wire KB/s.  Keys are sent as RFB KeyEvents (X11
keysyms), so single-core Zephyr doom is playable; the AMP doom build has no
keyboard path yet and ignores them.

usage: vnc_viewer.py HOST[:PORT] [--scale N] [--8bit | --rgb565] [--hextile] [--full] [--seconds N]
  --scale N    window zoom factor (default 3: 320x200 -> 960x600)
  --hextile    offer Hextile as well as Raw (decoded in Python: slower here)
  --full       non-incremental requests (full-frame service rate, like
               perf/amp_p1/vnc_probe.py --full)
  --8bit       8-bit colour-map pixels (default against "KlaussCPU core2")
  --rgb565     RGB565 true colour (default against any other server)
  --seconds N  close after N seconds and print the averages (for scripting)
"""
import argparse
import array
import socket
import struct
import sys
import threading
import time
import tkinter as tk

ENC_RAW, ENC_HEXTILE = 0, 5
HT_RAW, HT_BG, HT_FG, HT_SUB, HT_SUBCOL = 1, 2, 4, 8, 16
CORE2_NAME = "KlaussCPU core2"   # AMP core 2: supports the 8-bit colour map


def recv_exact(s, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = s.recv_into(view[got:], n - got)
        if k == 0:
            raise ConnectionError("server closed the connection")
        got += k
    return buf


# RGB565 value -> 3 bytes of RGB888, for the PPM handed to Tk.
LUT = []
for _v in range(65536):
    _r, _g, _b = (_v >> 11) & 0x1F, (_v >> 5) & 0x3F, _v & 0x1F
    LUT.append(bytes(((_r << 3) | (_r >> 2), (_g << 2) | (_g >> 4), (_b << 3) | (_b >> 2))))


class RfbClient:
    def __init__(self, host, port, hextile, full, fmt="rgb565"):
        """fmt: "rgb565", "8bit", or "auto" (8-bit against AMP core 2)."""
        self.full = full
        self.fmt = fmt
        self.palette = tuple(b"\0\0\0" for _ in range(256))   # 8-bit: index -> RGB
        self.s = socket.create_connection((host, port), timeout=10)
        self.s.settimeout(None)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.send_lock = threading.Lock()
        self.lock = threading.Lock()
        self.running = True
        self.error = None
        self._handshake(hextile)
        self.fb = bytearray(self.w * self.h * self.bpp)
        self.frame = None          # latest complete frame (bytes, palette), for the UI
        self.seq = 0               # incremented per complete update
        self.bytes_rx = 0

    def _send(self, data):
        with self.send_lock:
            self.s.sendall(data)

    def _handshake(self, hextile):
        s = self.s
        ver = bytes(recv_exact(s, 12))
        if not ver.startswith(b"RFB "):
            raise ConnectionError(f"not an RFB server: {ver!r}")
        minor = int(ver[8:11])
        if minor >= 7:
            s.sendall(b"RFB 003.008\n")
            n = recv_exact(s, 1)[0]
            if n == 0:
                ln = struct.unpack(">I", recv_exact(s, 4))[0]
                raise ConnectionError(recv_exact(s, ln).decode(errors="replace"))
            if 1 not in recv_exact(s, n):
                raise ConnectionError("server requires authentication (only 'None' is supported)")
            s.sendall(b"\x01")
            if struct.unpack(">I", recv_exact(s, 4))[0] != 0:
                raise ConnectionError("security handshake failed")
        else:
            s.sendall(b"RFB 003.003\n")
            if struct.unpack(">I", recv_exact(s, 4))[0] != 1:
                raise ConnectionError("server requires authentication (only 'None' is supported)")
        s.sendall(b"\x01")  # shared session
        si = recv_exact(s, 24)
        self.w, self.h = struct.unpack(">HH", si[:4])
        self.name = recv_exact(s, struct.unpack(">I", si[20:24])[0]).decode(errors="replace")
        if self.fmt == "auto":
            self.fmt = "8bit" if self.name == CORE2_NAME else "rgb565"
        if self.fmt == "8bit":
            # SetPixelFormat: 8 bpp colour map (true_colour = 0).
            self.bpp = 1
            s.sendall(struct.pack(">B3xBBBBHHHBBB3x", 0, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0))
        else:
            # SetPixelFormat: 16 bpp, depth 16, little-endian, true colour, RGB565.
            self.bpp = 2
            s.sendall(struct.pack(">B3xBBBBHHHBBB3x", 0, 16, 16, 0, 1, 31, 63, 31, 11, 5, 0))
        encs = [ENC_HEXTILE, ENC_RAW] if hextile else [ENC_RAW]
        s.sendall(struct.pack(">BxH", 2, len(encs)) + b"".join(struct.pack(">i", e) for e in encs))

    def request(self, incremental):
        self._send(struct.pack(">BBHHHH", 3, 1 if incremental else 0, 0, 0, self.w, self.h))

    def key(self, keysym, down):
        try:
            self._send(struct.pack(">BBxxI", 4, 1 if down else 0, keysym))
        except OSError:
            pass

    def _fill(self, x, y, rw, rh, pix):
        row = pix * rw
        for yy in range(y, y + rh):
            o = (yy * self.w + x) * self.bpp
            self.fb[o:o + rw * self.bpp] = row

    def _raw(self, x, y, rw, rh, data):
        if x == 0 and rw == self.w:
            o = y * self.w * self.bpp
            self.fb[o:o + len(data)] = data
            return
        stride = rw * self.bpp
        for r in range(rh):
            o = ((y + r) * self.w + x) * self.bpp
            self.fb[o:o + stride] = data[r * stride:(r + 1) * stride]

    def _hextile(self, x, y, rw, rh):
        s = self.s
        n = 0
        bg = fg = bytes(self.bpp)
        for ty in range(y, y + rh, 16):
            th = min(16, y + rh - ty)
            for tx in range(x, x + rw, 16):
                tw = min(16, x + rw - tx)
                sub = recv_exact(s, 1)[0]
                n += 1
                if sub & HT_RAW:
                    data = recv_exact(s, tw * th * self.bpp)
                    n += len(data)
                    self._raw(tx, ty, tw, th, data)
                    continue
                if sub & HT_BG:
                    bg = bytes(recv_exact(s, self.bpp)); n += self.bpp
                if sub & HT_FG:
                    fg = bytes(recv_exact(s, self.bpp)); n += self.bpp
                self._fill(tx, ty, tw, th, bg)
                if sub & HT_SUB:
                    cnt = recv_exact(s, 1)[0]
                    each = (self.bpp if sub & HT_SUBCOL else 0) + 2
                    data = recv_exact(s, cnt * each)
                    n += 1 + len(data)
                    for i in range(cnt):
                        p = data[i * each:(i + 1) * each]
                        pix = bytes(p[:self.bpp]) if sub & HT_SUBCOL else fg
                        xy, wh = p[-2], p[-1]
                        self._fill(tx + (xy >> 4), ty + (xy & 15), (wh >> 4) + 1, (wh & 15) + 1, pix)
        return n

    def run(self):
        s = self.s
        try:
            self.request(False)
            while self.running:
                t = recv_exact(s, 1)[0]
                if t == 0:  # FramebufferUpdate
                    nrects = struct.unpack(">xH", recv_exact(s, 3))[0]
                    total = 4
                    for _ in range(nrects):
                        x, y, rw, rh, enc = struct.unpack(">HHHHi", recv_exact(s, 12))
                        total += 12
                        if enc == ENC_RAW:
                            data = recv_exact(s, rw * rh * self.bpp)
                            total += len(data)
                            self._raw(x, y, rw, rh, data)
                        elif enc == ENC_HEXTILE:
                            total += self._hextile(x, y, rw, rh)
                        else:
                            raise ConnectionError(f"unsupported encoding {enc}")
                    # Ask for the next update before handing this one to the UI,
                    # so the server works while we convert and draw.
                    self.request(not self.full)
                    with self.lock:
                        self.frame = (bytes(self.fb), self.palette)
                        self.seq += 1
                        self.bytes_rx += total
                elif t == 1:  # SetColourMapEntries (8-bit colour map)
                    first, n = struct.unpack(">xHH", recv_exact(s, 5))
                    ent = recv_exact(s, n * 6)
                    pal = list(self.palette)
                    for i in range(n):
                        if first + i < 256:
                            r, g, b = struct.unpack_from(">HHH", ent, i * 6)
                            pal[first + i] = bytes((r >> 8, g >> 8, b >> 8))
                    self.palette = tuple(pal)   # frames keep the palette they arrived with
                elif t == 2:  # Bell
                    pass
                elif t == 3:  # ServerCutText
                    recv_exact(s, 3)
                    recv_exact(s, struct.unpack(">I", recv_exact(s, 4))[0])
                else:
                    raise ConnectionError(f"unknown server message {t}")
        except (OSError, ConnectionError) as e:
            if self.running:
                self.error = str(e)
        finally:
            self.running = False

    def close(self):
        self.running = False
        try:
            self.s.close()
        except OSError:
            pass


class Viewer:
    def __init__(self, client, scale, seconds):
        self.c = client
        self.scale = scale
        self.root = tk.Tk()
        self.root.title(f"{client.name} — connecting")
        self.root.resizable(False, False)
        self.src = tk.PhotoImage(width=client.w, height=client.h)
        self.disp = tk.PhotoImage(width=client.w * scale, height=client.h * scale)
        tk.Label(self.root, image=self.disp, borderwidth=0, bg="black").pack()
        self.root.bind("<KeyPress>", lambda e: self.c.key(e.keysym_num, True))
        self.root.bind("<KeyRelease>", lambda e: self.c.key(e.keysym_num, False))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.ppm_hdr = b"P6 %d %d 255\n" % (client.w, client.h)
        self.shown_seq = 0
        self.t_start = time.time()
        self.win_t, self.win_seq, self.win_shown, self.win_bytes = self.t_start, 0, 0, 0
        self.total_shown = 0
        self.first_seq = None
        if seconds:
            self.root.after(int(seconds * 1000), self.close)
        self.root.after(5, self.tick)

    def tick(self):
        c = self.c
        with c.lock:
            frame, seq, nbytes = c.frame, c.seq, c.bytes_rx
        if seq != self.shown_seq and frame is not None:
            pixels, pal = frame
            if c.bpp == 1:
                rgb = b"".join(map(pal.__getitem__, pixels))
            else:
                arr = array.array("H")
                arr.frombytes(pixels)
                if sys.byteorder != "little":
                    arr.byteswap()
                rgb = b"".join(map(LUT.__getitem__, arr))
            self.src.configure(data=self.ppm_hdr + rgb, format="PPM")
            self.disp.tk.call(self.disp, "copy", self.src, "-zoom", self.scale, self.scale)
            self.shown_seq = seq
            self.win_shown += 1
            self.total_shown += 1
            if self.first_seq is None:
                self.first_seq = (seq, nbytes, time.time())
        now = time.time()
        if now - self.win_t >= 2.0:
            dt = now - self.win_t
            self.root.title(
                f"{c.name}  {c.w}x{c.h} {'8-bit' if c.bpp == 1 else 'RGB565'} — recv {(seq - self.win_seq) / dt:.1f} fps, "
                f"shown {self.win_shown / dt:.1f} fps, {(nbytes - self.win_bytes) / dt / 1024:.0f} KB/s")
            self.win_t, self.win_seq, self.win_shown, self.win_bytes = now, seq, 0, nbytes
        if not c.running:
            self.root.title(f"{c.name} — disconnected: {c.error or 'closed'}")
            return
        self.root.after(5, self.tick)

    def close(self):
        c = self.c
        with c.lock:
            seq, nbytes = c.seq, c.bytes_rx
        if self.first_seq:
            s0, b0, t0 = self.first_seq
            dt = time.time() - t0
            if dt > 0 and seq > s0:
                print(f"{seq - s0} updates in {dt:.1f}s = {(seq - s0) / dt:.2f} fps received, "
                      f"{self.total_shown / dt:.2f} fps shown, "
                      f"avg {(nbytes - b0) // (seq - s0)} B/update, {(nbytes - b0) / dt / 1024:.0f} KB/s")
        if c.error:
            print(f"disconnected: {c.error}")
        c.close()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description="Minimal VNC viewer for the KlaussCPU board")
    ap.add_argument("host", help="board address, e.g. 192.168.68.59 or 192.168.68.59:5900")
    ap.add_argument("--port", type=int, default=5900)
    ap.add_argument("--scale", type=int, default=3)
    fmt = ap.add_mutually_exclusive_group()
    fmt.add_argument("--8bit", dest="fmt", action="store_const", const="8bit")
    fmt.add_argument("--rgb565", dest="fmt", action="store_const", const="rgb565")
    ap.add_argument("--hextile", action="store_true")
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--seconds", type=float, default=0)
    a = ap.parse_args()
    host, port = a.host, a.port
    if host.count(":") == 1:
        host, p = host.split(":")
        port = int(p)
    elif "::" in host:
        host, p = host.split("::")
        port = int(p)
    try:
        client = RfbClient(host, port, a.hextile, a.full, a.fmt or "auto")
    except (OSError, ConnectionError) as e:
        sys.exit(f"vnc_viewer: cannot connect to {host}:{port}: {e}")
    print(f"connected: '{client.name}' {client.w}x{client.h}, "
          f"{'8-bit colour map' if client.bpp == 1 else 'RGB565'}, "
          f"encodings {'hextile+raw' if a.hextile else 'raw'}")
    viewer = Viewer(client, a.scale, a.seconds)
    threading.Thread(target=client.run, daemon=True).start()
    viewer.root.mainloop()


if __name__ == "__main__":
    main()
