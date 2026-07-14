#!/usr/bin/env python3
# kbt2mem.py — convert a klausscc .kbt (board wire image) to a boot_rom $readmemh
# netboot.mem (one little-endian 64-bit doubleword per line, 16 hex digits).
#
# .kbt layout:  'S' <image-hex, uppercase, LE flat DDR image incl. heap header> 'Z' <cksum> 'X'
# The image is byte-identical to `klausscc --mem-out <elf>` output — this lets a
# HAND-ASSEMBLED .kla (which --mem-out cannot consume) feed tb_soc / tb_pipeline_isa.
import sys
kbt = open(sys.argv[1]).read().strip()
assert kbt[0] == 'S', "kbt must start with 'S'"
hexchars = '0123456789ABCDEFabcdef'
body = kbt[1:]
n = 0
while n < len(body) and body[n] in hexchars:
    n += 1
if n % 2:                      # defensive: image hex must be whole bytes
    n -= 1
img = bytes.fromhex(body[:n])
# word0 lo32 = heap_start = total image byte length; drop the trailing entry-point
# doubleword the wire format appends past the image (matches --mem-out exactly).
if len(img) >= 8:
    byte_len = int.from_bytes(img[0:4], 'little')
    if 0 < byte_len <= len(img):
        img = img[:byte_len]
out = sys.argv[2] if len(sys.argv) > 2 else '/dev/stdout'
with open(out, 'w') as f:
    for i in range(0, len(img), 8):
        chunk = img[i:i+8].ljust(8, b'\x00')
        f.write('%016X\n' % int.from_bytes(chunk, 'little'))
