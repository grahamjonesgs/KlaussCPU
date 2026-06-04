# NETBOOT_PLAN — Load programs over Ethernet (UDP) instead of UART kbt

Goal: load a flat program image into DDR over the LAN and run it, replacing the
slow ELF→kbt-ASCII→3 Mbaud-UART path. Target: ~1 MB image in well under a second,
one host command (`elf in → running on board`).

Decided context (see discussion that produced this plan):

- Host (Mac on WiFi) and the FPGA are on **one subnet** `192.168.68.0/24`, bridged
  through the Deco mesh + a dumb switch. **No L3 router hop** between them, so the
  board is reachable at L2 — but we go **IP/UDP**, not raw L2, because (a) you
  already know the board's IP from the Deco DHCP reservation, and (b) UDP/IP
  bridges across the WiFi mesh reliably where custom EtherTypes / WiFi broadcast
  do not.
- LiteEth here is a **raw MAC** (`tools/liteeth/liteeth_nexys_a7.yml`, `core: wishbone`)
  — no hardware IP/UDP. **But there is already a working lwIP NO_SYS=1 port** in
  the LLVM runtime (`runtime/programs/lwip_demo.c`, `lwip_port/ethernetif.*`,
  `src/eth.h`) doing DHCP + ICMP + **TCP**. That changes everything below: we do
  **not** hand-roll ARP/UDP — we receive the image over a **single TCP connection**
  and let TCP handle ordering / loss / retransmit / flow control for free.
- Board MAC is **`00:AB:CD:00:00:01`** (set in software by `eth_init`/the runtime,
  overriding the LiteEth YAML default `0x020000000001`). The Deco reservation maps
  this MAC → **`192.168.68.59`**.
- Chosen architecture: **resident C bootloader** (not a hardware Wishbone-mastering
  FSM). All the hard parts — PHY bring-up, ARP, UDP, loss/retransmit — are reused
  or trivial in C; they would be miserable in RTL.

---

## 1. What we reuse (already on the board)

| Asset | Where | Use |
|---|---|---|
| DDR write port | `r_mem_addr` / `r_mem_write_data` / `r_mem_byte_en` / `r_mem_write_DV` / `w_mem_ready` ([KlaussCPU.v](KlaussCPU.srcs/sources_1/new/KlaussCPU.v)) | The bootloader writes the received image to DDR with ordinary STORE instructions — same path the UART loader uses. |
| LiteEth MAC + slot RX/TX | [eth_mmio_bridge.v](KlaussCPU.srcs/sources_1/new/eth_mmio_bridge.v), CSRs at `0xF006_xxxx`, slot SRAM at `0xF008_xxxx` ([MMIO_MAP.md:289](MMIO_MAP.md#L289)) | Receive frames, transmit ARP/ACK replies. |
| MDIO bit-bang + frame TX/RX in C | [eth_test.c](eth_test.c) (`mdio_read/write`, `test_passive_rx`, `test_external_tx`) | Starting point for PHY bring-up, RX drain loop, and ARP frame construction. |
| Program memory model | heap header at byte `0x00` (4 words), code at `0x20`, entry PC | The downloaded image is laid out exactly like today's kbt payload **minus the `S…ZX` framing** — so it loads to DDR at byte 0 and runs at `0x20`, unchanged. |
| Host ELF→flat | `parse_elf_to_flat()` ([klausscc/src/main.rs:551](../klausscc/src/main.rs)) | Already extracts LOAD segments + entry; the new sender reuses it verbatim. |
| IFB / cache coherence | store-into-buffered-line invalidation ([KlaussCPU.v:1271](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L1271)) | Guarantees the freshly-written program is fetched correctly when we jump to it (same property the Zephyr LLEXT loader relies on). |

---

## 2. The one hard problem: getting the bootloader to run with no program loaded

Today the CPU has nothing to execute at reset — it idles in `NO_PROGRAM` (the RGB
boot animation) until a UART break + `S`. DDR2 is external (MIG-controlled) and
cannot be `$readmemh`-initialized, so we cannot simply preload code into DDR.

**Solution: a small initialized "boot SRAM" BRAM the CPU runs from at reset.**

- New BRAM, ~16–32 KiB, `$readmemh`-initialized with the assembled bootloader
  (`netboot.mem`), holding bootloader **code + .data + stack** so the bootloader
  needs no DDR for its own working set.
- Mapped at a fixed base, proposed **`0xE000_0000`** (selector `addr[31:28]==4'hE`,
  currently routed to DRAM, so it's free to carve out).
- DDR (`0x0000_0000`–`0x07FF_FFFF`) stays entirely free as the **target** for the
  downloaded program — loads at `0x0`, runs at `0x20`, identical to a UART load.

This keeps the program memory model unchanged; the only new RTL is the boot SRAM
and routing fetch/data accesses for the `0xE` region to it.

---

## 3. Phasing (de-risk software before touching RTL)

### Phase 0 — Network + protocol proof, **zero RTL** ✅ IMPLEMENTED
Receiver built as an **ordinary UART-loaded test program**, reusing the existing
lwIP port — see [tools/netboot/netboot.c](tools/netboot/netboot.c) (copy into
`runtime/programs/` to build, it shares lwip_demo.c's includes):
1. `eth_init()` + `lwip_init()` + `netif_add(...)` — identical bring-up to lwip_demo.c.
2. DHCP (so the Deco reservation gives the known IP); static `192.168.68.50` fallback.
3. Listen on TCP port 5000; stream the image (protocol §4) into a **DDR staging
   region** `0x0400_0000`.
4. Verify the whole-image checksum, show it on the 7-seg, reply to the host.
5. **Does not jump yet** (that's Phase 1).

Host side built too: `klausscc --net-load <elf> --ip <board> [--port 5000]`
(reuses `parse_elf_to_flat` + new `build_ddr_image`, sends over TCP via
`src/netload.rs`). This proves PHY/DHCP/TCP over your real Deco+switch network,
independent of the RTL work.

### Phase 1 — Relocate + run (still UART-bootstrapped) ✅ IMPLEMENTED
The downloaded program is linked to run at `0x20` (where netboot itself runs), so
copying staging→`0x0` clobbers the copier mid-flight. Solved with a **trampoline**:
- [tools/netboot/trampoline.kla](tools/netboot/trampoline.kla) — 26-word copy-down
  routine (`MEMGET64`/`MEMSET64` loop, `SETSP 0x0800_0000`, `JMPR` entry), assembled
  with `klausscc` at base `0x20` to get verified machine code.
- `netboot.c` embeds those words as `TRAMP_TEMPLATE`, and on receive-complete
  `launch_image()` emits them to **`0x07FF_0000`** (high DDR, clear of the staging
  source, the `0x0` destination, and the stack), patching the len/entry immediates
  and relocating the two absolute jump targets by `TRAMP_BASE − 0x20`. It then
  calls into the trampoline, which copies the image to `0x0`, restores
  `SP = 0x0800_0000` (matching the HW loader), and jumps to the entry — so the
  program runs exactly as if kbt-loaded. Image cap 16 MiB keeps the regions disjoint.
- The jump is deferred to netboot's main loop (after a ~100 ms lwIP service window)
  so the TCP ACK/FIN flush before netboot overwrites itself. `NETBOOT_LAUNCH 0`
  reverts to Phase-0 verify-only.

Hardware-dependent risks to watch on first run (can't be tested off-board): the
C function-pointer call must lower to `CALLR <addr>`, and a 32-bit store must land
in memory in the byte order instruction fetch expects (both reasoned correct — the
kbt loader stores opcodes LE and the LLEXT loader relies on the same write-then-fetch
coherence).

### Phase 2 — Make it resident (RTL) ✅ DONE (tested on hardware)
Power-on lands directly in netboot (DHCP → IP `192.168.68.59` on the 7-seg), and
`klausscc --net-load <elf> --ip 192.168.68.59` loads and runs a program over the
LAN with no UART step. All three phases verified end-to-end.
**Revised approach (simpler than the original run-from-BRAM idea):** don't relink
netboot to run from BRAM — reuse the existing "copy image into DDR, then run" path.
A boot **ROM** holds netboot's *normal* image (built as today, linked at `0x20`,
**no relink, no crt0, no fetch-mux**); at reset a small hardware copy blits it into
DDR and hands off to `LOAD_COMPLETE`. netboot then runs from DDR identically to a
UART load — trampoline and all. (Credit: the working trampoline showed the
copy-into-place mechanism was all we needed.)

BRAM budget confirmed fine: 31.5/135 tiles used, ~103 free; netboot's ~183 KB image
is ~41 RAMB36 tiles. Watch timing, not capacity.

Implemented RTL (untested off-board — no Vivado here):
- [boot_rom.v](KlaussCPU.srcs/sources_1/new/boot_rom.v) — `$readmemh "netboot.mem"`,
  64-bit doublewords, 1-cycle read. Read-only; netboot's data lives in DDR as today.
- `KlaussCPU.v` — `boot_rom` instance + a copy sub-FSM inside `NO_PROGRAM`
  (`r_boot_phase`): waits `calib`, reads word 0 (`heap_start` = image length),
  writes each doubleword to DDR via the existing `r_mem_*` port, then sets
  `r_PC_requested=0x20` / equal checksums and jumps to `LOAD_COMPLETE`. Empty ROM
  (word 0 == 0) ⇒ normal idle. Shows `b00t` marker on the 7-seg while copying.
- `calib_done` plumbed up: `ddr2_control.v` → `mem_read_write.v` → `KlaussCPU.v`.
- [klausscc](../klausscc) `--mem-out <elf>` — emits `netboot.mem` (verified here).
- UART break+`S` still preempts anytime (handler sits above the FSM), so reflash works.

To bring up (you, Vivado/Mac):
1. `klausscc --mem-out netboot.elf --mem-file netboot.mem` (your existing netboot build).
2. Add `boot_rom.v` and `netboot.mem` to the Vivado project (so `$readmemh` finds it).
3. Synth/impl; check timing with the added BRAM. Power on → `b00t` flickers →
   netboot's `INIT`/`dHCP`/IP — no UART step.

Risks to watch (untestable off-board): the boot copy's cache-write handshake
(mirrors the proven UART `LOADING_BYTE` path), `calib` gating, and that netboot's
entry is `0x20` (confirmed from your load logs).

---

## 4. Wire protocol (TCP — lwIP does the hard part)

Because the board already runs lwIP TCP, the loss-tolerant UDP+NACK scheme this
plan originally proposed is **unnecessary** — TCP already guarantees in-order,
reliable, flow-controlled delivery, which is exactly what WiFi packet loss and the
2-slot RX overflow need. The board opens a TCP listener; the host connects and
streams:

```
[ 12-byte header, little-endian ]
    u32 magic    = 0x5445_4E4B   ("KNET")
    u32 img_len  = bytes of DDR image that follow
    u32 entry_pc = board byte address to jump to (e.g. 0x20)
[ img_len bytes ] = the full DDR image: 4×64-bit heap header @0x00 + code @0x20,
                    i.e. the kbt payload layout minus S/Z/X framing.
```

Board reply (8 bytes, little-endian): `u32 status` (0 = OK), `u32 checksum`
(sum of LE 32-bit words — host computes the same and compares, belt-and-braces
on top of TCP's own integrity).

No sequence numbers, no NACK, no bitmap, no per-block ACK stalls. The board's
TCP receive callback simply appends each `pbuf` to the staging buffer until
`img_len` bytes have arrived. If throughput on the NO_SYS poll loop proves
inadequate for very large images we can revisit a UDP blast, but TCP is the
correct default and is what Phase 0 implements.

---

## 5. Host side — `klausscc --net-load` ✅ IMPLEMENTED

Pure additive Rust; the existing kbt/UART path is untouched.

- `klausscc --net-load <elf|bin> --ip <board_ip> [--port 5000] [--entry 0x..]`.
- Reuses `parse_elf_to_flat()` for `(flat_image, entry_pc)` — same as `elf2serial`.
- New `build_ddr_image()` (helper.rs) wraps the flat image in the heap-header DDR
  layout as raw LE bytes (no ASCII-hex, no framing).
- `src/netload.rs`: `TcpStream::connect`, send 12-byte header + image, read the
  8-byte ack, compare checksum.
- Ordinary `std::net` TCP to the reserved IP — no raw sockets, no MAC handling.

---

## 6. RTL changes for Phase 2 (boot SRAM)

Minimal and contained. Integration points:

1. **New module `boot_sram.v`** — a BRAM returning a 64-bit doubleword for a
   doubleword address in 1 cycle, `$readmemh`-initialized from `netboot.mem`.
   Make it writable (so .data/.bss/stack live here) — i.e. initialized RAM, not ROM.
   A dual-port BRAM (port A = fetch by `r_PC`, port B = data by `r_mem_addr`) avoids
   contention between instruction fetch and the bootloader's own loads/stores.
2. **Decode in `bus_splitter.v`** — add a third destination `is_boot = (addr[31:28]==4'hE)`
   alongside `is_eth`/`is_mmio`, routing those accesses to `boot_sram` instead of the
   cache. (`0xE` currently falls through to DRAM.)
3. **Mux in the fetch/data path (`KlaussCPU.v`)** — when the active address is in the
   boot region, source the opcode doubleword, load-read-data, and `w_mem_ready` from
   `boot_sram` instead of the cache. Simplest: **bypass the IFB for the boot region**
   (boot SRAM is already single-cycle; the bootloader isn't self-modifying).
4. **Reset behaviour** — after MIG `init_calib_complete`, set `r_PC <= 32'hE000_0000`
   and `r_SM <= OPCODE_REQUEST` (run the bootloader) instead of idling in `NO_PROGRAM`.
   Keep the UART break→`S` path as an override so you can still flash a new bootloader.
   Gate the start on calib because the bootloader writes the payload to DDR.

Risk note: the fetch path has the IFB + dual-doubleword read (`w_ifb_op_dw`); the mux
in (3) is the trickiest bit and the main thing to test on hardware (ILA on `r_PC`,
`r_mem_addr`, `w_mem_ready`). Bypassing IFB for `0xE` keeps it simple.

Alternative resident sources considered and rejected for now: SD-card boot
(`sd_spi.v` exists, but adds an SPI-sector FSM and an SD dependency); preloading DDR
(impossible — external MIG memory).

---

## 7. Board-side config (constants in `netboot.c`)

- `BOARD_MAC = 00:AB:CD:00:00:01` (set by `eth_init`/runtime; Deco reservation
  maps it to `192.168.68.59`).
- **DHCP first** — the reservation hands the board `192.168.68.59` automatically.
  `STATIC_IP = 192.168.68.59` is the fallback if DHCP times out (10 s), so the
  board lands on the same address either way.
- `NETBOOT_PORT = 5000` — must match `--port` on the host.

---

## 8. Decisions

1. **Header ownership** — ✅ **host owns the layout**: `build_ddr_image` prepends the
   4-word heap header and patches `heap_start`; the board blits the image to DDR and
   gets `entry_pc` from the protocol header. Board stays a dumb sink.
2. **Reliability** — ✅ **TCP** (lwIP), not UDP+NACK. Revisit only if NO_SYS TCP
   throughput is inadequate for large images.
3. **Addressing** — ✅ **DHCP** (the Deco reservation yields the known IP), static
   `192.168.68.50` fallback. No board-side discovery needed — host targets the IP.
4. **Boot SRAM size (Phase 2 only)** — ⚠️ lwIP is large; 16 KiB will **not** fit the
   full stack. For the resident bootloader either grow the boot BRAM (e.g. 64–128 KiB)
   or strip lwIP to TCP-only / hand-roll a minimal stack. Doesn't affect Phase 0/1
   (UART-loaded, runs from DDR). Decide when starting Phase 2.

---

## 9. Suggested build order

1. Host `netload` sender + `netboot.c` receiver to **staging**, checksum-only
   (Phase 0) — prove the network end-to-end. ← highest-risk, do first.
2. Add relocate+jump (Phase 1) — prove netboot runs a real program.
3. `boot_sram.v` + decode + fetch mux + run-at-reset (Phase 2) — make it resident.
4. Keep UART kbt loader throughout as fallback / bootloader-flash path.
