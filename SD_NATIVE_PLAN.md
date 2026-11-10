# Native 4-bit SD controller: plan

Status: **register map frozen (v1, 2026-11-10); phase 1 RTL in progress** on
klaussCPU branch `mmu-sd` (Graham's go-ahead, relayed by the runtime session).
Plan written 2026-11-10. Requested by Graham through the
klausscpu-runtime session; that session reviews the software interface (§5)
and plans the Linux, Zephyr and bare-metal driver changes.

## 1. Why

Linux now mounts its root file system from the SD card, an ext4 image on the
FAT partition, and the card is the bottleneck. Writing a 96 MiB image takes
more than 10 minutes, about 0.16 MB/s.

Today's interface (§2) is SPI mode. Every byte costs one MMIO write, a poll
and one MMIO read, 32 fabric cycles of shifting, and CPU time. The ceilings:

| Interface | Raw bus rate | Expected sustained |
|---|---|---|
| SPI 25 MHz, byte loop (today) | 3.1 MB/s | ~0.16 MB/s measured (single-block, polled) |
| SPI 25 MHz, CMD18/CMD25 software port | 3.1 MB/s | ~1.5–2.5 MB/s, all CPU |
| **SD bus, 4-bit, default speed 25 MHz** | 12.5 MB/s | ~11 MB/s read; writes limited by the card |
| **SD bus, 4-bit, high speed 50 MHz** | 25 MB/s | ~20 MB/s read; writes limited by the card |

The software port of CMD18/CMD25, already under way, is the cheap first
step, worth roughly 10×. The native controller is worth another 4–8× on top
and takes the CPU out of the bit loop.

UHS-I modes (SDR50/SDR104) are out of reach. The Nexys A7 slot is 3.3 V
only, with no 1.8 V signalling switch, so high speed at 50 MHz is the
ceiling.

## 2. What exists today

`sd_spi.sv`, MMIO base `0xF000_0000` (device id 0x000), `MMIO_MAP.md`
"SD card":

- `SD_CTRL` (0x000): `[15:0]` clk_div, `[16]` cs_n, `[17]` pwr_en. SCK =
  100 MHz / (2·(clk_div+1)). The reset value 249 gives 200 kHz; 1 gives
  25 MHz.
- `SD_DATA` (0x008): writing it shifts one byte, and reading it returns the
  last byte received.
- `SD_STATUS` (0x010): `[0]` busy, `[1]` card_present (the raw `SD_CD` pin).
- **`SD_BUF` (0x200–0x3F8), the "SD_BUF_PTR" question.** This is 512 bytes of
  plain LUTRAM, read and written 64 bits at a time. Nothing in the hardware
  connects it to the shifter, so it is just scratch RAM at an MMIO address.
  No driver uses it, because copying through it saves nothing. It was meant
  for a block engine that never got built.

What the SPI design assumes:
- SPI mode 0, MSB first, chip select driven by software;
- the whole SD protocol in software (CMD0/8/55/41/58/17/24, the 0xFE token,
  R1b busy polling, CRC off);
- the slot powered off at reset, through the `SD_RESET` FET, active low;
- DAT1 and DAT2 driven high;
- every SD pin except DAT0 an output only.

A card that has been put into SPI mode (CMD0 with CS low) stays in SPI mode
until it is power-cycled.

## 3. Pins (question 1)

All of them reach the FPGA. These are in `nexys_ddr.xdc`, all in bank 35 at
LVCMOS33:

| Signal | Pin | Today's port | SD-mode use |
|---|---|---|---|
| SD_SCK | B1 | `o_SD_SCK` | CLK |
| SD_CMD | C1 | `o_SD_MOSI` | CMD (bidirectional) |
| SD_DAT0 | C2 | `i_SD_MISO` | DAT0 (bidirectional, busy) |
| SD_DAT1 | E1 | `o_SD_DAT1` | DAT1 (bidirectional) |
| SD_DAT2 | F1 | `o_SD_DAT2` | DAT2 (bidirectional) |
| SD_DAT3 | D2 | `o_SD_CS_n` | DAT3 (bidirectional) |
| SD_CD | A1 | `i_SD_CD` | card detect |
| SD_RESET | E2 | `o_SD_RESET` | slot power, active low |

The RTL change is to turn CMD and DAT[3:0] into `inout` ports with IOBUFs.
In SPI mode each pin keeps its current direction and value, so SPI behaviour
does not change.

The XDC gains `PULLUP true` on CMD and DAT[3:0], since the SD bus needs
pull-ups. That is harmless if the board already has them; the Digilent
reference manual's figure shows external pull-ups, but this hasn't been
checked against the schematic. It also gains `IOB TRUE` on the output and
input flip-flops.

## 4. Design

### 4.1 Mode and compatibility (question 5)

- New `SD_CTRL[18]` = **SD_MODE**, reset 0. With 0, the block is exactly
  today's SPI controller: same registers, same pins, same timing.
  - Old software writes `SD_CTRL` as a whole word, so it can only ever
    select SPI mode.
  - Zephyr, the bare-metal loader, `test_sd` and netboot don't change.
- `clk_div` and `pwr_en` are shared between the modes. In SD mode, CLK =
  100 MHz / (2·(clk_div+1)): 124 gives 400 kHz for identification, 1 gives
  25 MHz, and 0 gives 50 MHz (high speed, §4.6).
- Switching modes needs a power cycle, which is a software rule.
  - After anything has used SPI mode (the boot ROM, netboot, the loader), the
    card is stuck in SPI mode. To move it to SD mode: set SD_MODE, drop
    `pwr_en` for ≥ 10 ms (to be measured on the board: the slot's bulk
    capacitance decides how long), power up, run ≥ 74 clocks with CMD high
    (`SDM_CFG.CLK_ON`), then CMD0 with DAT3 high, CMD8, ACMD41,
    CMD2, CMD3, CMD7, ACMD6 (4-bit), and optionally CMD6 for high speed.
  - Linux does this in its `set_ios` / `power_up` path, as the mmc core
    expects.
- **Reset and program load** (decided after the runtime review, point 3):
  - Today `sd_spi` resets only on `CPU_RESETN`. A klausscc reload (UART
    break or `S`, or netboot) goes through `LOAD_COMPLETE`, which leaves
    `SD_CTRL` alone, so the slot stays powered with its clock divider and
    chip select unchanged.
  - **CPU_RESETN** resets everything, as now: SD_MODE 0, `pwr_en` 0, the
    engines idle, the FIFO empty, the `SDM_*` registers at their reset
    values.
  - **LOAD_COMPLETE with SD_MODE = 1** (the last program used native
    mode): the controller resets as on CPU_RESETN. That clears SD_MODE,
    **clears `pwr_en`**, cuts the slot's power, aborts any transfer and
    empties the FIFO. The new program starts in SPI mode with the card
    unpowered, so its SPI init begins from a power-up.
  - **LOAD_COMPLETE with SD_MODE = 0**: nothing changes, exactly as today,
    so existing SPI flows that leave the card initialised across a load
    behave the same.
  - **Rule for SPI drivers** (Zephyr, the loader, `test_sd`): before the
    first CMD0, write `pwr_en` = 0, wait for the power-off time (≥ 10 ms,
    to be measured), then power up. This covers both a load straight after
    native mode and a card left in an unknown state. Without that wait, the
    slot is only off for as long as the new program takes to reach its SD
    init, which may be too short to reset the card. As a second line of
    defence, CMD0 with CS low is accepted in every SD-mode state except
    inactive and switches the card to SPI mode, so today's init usually
    recovers anyway. The power cycle makes it deterministic.
- **Discovery** has two parts, both reading 0 on older bitstreams:
  - `CPU_FEATURES[21]` = native SD controller present, the same scheme as
    the crypto bits (MMU_PLAN §6.3; agreed with the runtime session);
  - `SDM_CAPS` (0x1F8) for the details: version, FIFO size, high speed
    supported.
- `SD_BUF` (0x200–0x3F8) stays as scratch RAM for compatibility. It costs
  about 64 LUTs. It could be deleted, since no software uses it, but keeping
  it costs nothing in risk.

### 4.2 Command engine (question 4)

The CPU writes `SDM_ARG`, then `SDM_CMD`, which starts the command:

- `[5:0]` command index;
- `[10:8]` response type:
  - 0 none;
  - 1 48-bit with CRC (R1, R5, R6, R7);
  - 2 48-bit with CRC plus busy (R1b);
  - 3 136-bit (R2, CRC over the CID/CSD payload);
  - 4 48-bit with no CRC check (R3, whose CRC field is 0x7F).
- `[11]` data phase follows;
- `[12]` data direction, 1 = write;
- `[13]` stop or abort: CMD12. It ends a running open-ended data phase
  cleanly, and the busy that follows is handled as for R1b. Software writes
  index 12, type 2 (R1b) and `[13]` = 1. A read block that is only partly
  received is rolled back (§4.3).

Sequencing rules:
- **Read data:** the data engine starts looking for the start bit as soon as
  the command's end bit has been sent, because the card may start data
  before its response has finished.
- **Write data:** the data engine starts after a good response, at least
  2 clocks (N_WR) after the response's end bit.
- **Response error:** a CRC, end-bit, index or timeout error on a data
  command aborts its data phase. DATA_BUSY clears, and DATA_DONE is not set.
- **CMD_DONE** is set when every command finishes, errors included. For
  R1b it is set after the busy ends. The error bits are set alongside it.

Hardware:
- sends the 48-bit frame with CRC7 generated serially;
- waits up to N_CR = 64 clocks for the start bit;
- captures the 48- or 136-bit response;
- checks the CRC7 (not for type 4), the end bit, and that the echoed command
  index matches (types 1 and 2).

Results:
- `SDM_RESP0`/`SDM_RESP1` hold the response.
  - **48-bit response:** `RESP0[31:0]` = the 32-bit argument field (card
    status, OCR, RCA), and `RESP0[37:32]` = the echoed index field.
  - **R2:** RESP1:RESP0 = the raw response bits [127:0], including the
    CRC7 and the end bit in `RESP0[7:0]`. The CRC7 is checked over bits
    [127:8].
  - **Linux:** `resp[0]` = `RESP1[63:32]`, `resp[1]` = `RESP1[31:0]`,
    `resp[2]` = `RESP0[63:32]`, `resp[3]` = `RESP0[31:0]`. That is the
    layout the mmc core expects, with the CRC byte in the low byte of
    `resp[3]`.
- For R1b, the engine keeps CMD_BUSY until DAT0 goes high, or until the busy
  timeout.
- Errors (sticky, write-1-to-clear in `SDM_STATUS`): CMD_TIMEOUT,
  CMD_CRC_ERR, CMD_END_ERR, CMD_IDX_ERR.

### 4.3 Data engine

- `SDM_BLK`:
  - `[11:0]` block size in bytes, **a multiple of 8, from 8 to 2048**: 512
    for blocks, 64 for the SD status and the CMD6 switch status, 8 for the
    SCR. Bits `[2:0]` are ignored on write and read back as 0.
  - `[31:16]` block count, where 0 means open-ended until a CMD12.
- `SDM_BLKDONE` (0x158, read-only) counts **good blocks** in the current
  data command. It resets to 0 when an `SDM_CMD` with the data flag is
  written.
  - A read block counts when it is in the FIFO with a good CRC and end bit.
  - A write block counts when the card's CRC token was 010 and its busy has
    ended.
  - So Linux's `bytes_xfered` = `SDM_BLKDONE` × block size, on success and
    after an error.
- Bus width: `SDM_CFG[0]`, 0 = 1-bit (DAT0 only, used during
  identification), 1 = 4-bit.
- **DATA_DONE** is set only on success. On an error the error bit is set
  and DATA_BUSY clears.
- **Pins in SD mode:**
  - CMD is driven only while a command is being sent, and is otherwise an
    input.
  - DAT[3:0] are driven only while write data and its CRC are being sent,
    and are otherwise inputs. `SD_CTRL[16]` (cs_n) is ignored.
- **Clock:**
  - with `SDM_CFG[2]` (CLK_ON) = 1, CLK runs freely; that is how the 74
    init clocks are given;
  - otherwise CLK runs only during a command, a response, a data phase or a
    busy wait, plus 8 trailing clocks (N_RC/N_CC), and while the FIFO flow
    control stops it (§4.3).
- **Read:**
  - for each block: a start bit on all active lines, the payload nibbles,
    one CRC16 per line (four serial CRC16 LFSRs), and the end bit;
  - errors: DATA_CRC_ERR, DATA_END_ERR, and DATA_TIMEOUT (`SDM_TIMEOUT`,
    in CLK cycles; the default is about 100 ms);
  - **Byte order:** each FIFO word holds the next 8 bytes of the block,
    little-endian. Byte 0 of the block is in bits [7:0] of the first word,
    byte 7 in [63:56], byte 8 in [7:0] of the second word. A 64-bit load
    stored to RAM therefore gives the block in memory order with no
    swapping. Pushes for a write use the same layout.
  - **A block with a CRC or end-bit error never reaches the FIFO.** Words
    are written behind a shadow write pointer that only becomes visible when
    the block checks good; a bad block is rolled back. The data engine then
    stops: DATA_BUSY clears, CLK stops, the error bit is set, and the driver
    sends CMD12. The good blocks before it stay in the FIFO, and
    `SDM_BLKDONE` counts them.
- **Write:**
  - a block is sent only when it is complete in the FIFO;
  - then the card's CRC status token on DAT0 is checked:
    - 010 is OK;
    - any other token value raises **WRITE_CRC_ERR**;
    - no token start bit within 8 clocks of the end bit raises
      **WRITE_NO_TOKEN**;
  - then the engine waits for busy on DAT0 before the next block. If busy
    outlasts `SDM_TIMEOUT[63:32]` (default about 500 ms), that raises
    **BUSY_TIMEOUT**;
  - on any of these errors the block is not counted in `SDM_BLKDONE`, and
    the engine stops as on a read error. The driver sends CMD12 and resets
    the FIFO (`SDM_FIFO[31]`) to drop the unsent data.
  - DAT0 is also readable live, as `SDM_STATUS.DAT0`.
- **Flow control by stopping the clock:**
  - When the FIFO cannot take another whole block (on a read) or does not
    yet hold one (on a write), the engine stops CLK between blocks. The
    spec allows this, and SDHCI does the same.
  - The CPU, or a slow interrupt handler, can never overrun or underrun the
    card. It only slows the transfer.
- **DATA_DONE** is set when the block count is exhausted. For a read that
  is when the last block is in the FIFO; for a write, when the last block's
  busy has ended.
  - For open-ended transfers and for error recovery, the driver issues
    CMD12 (`SDM_CMD[13]`).
  - This matches the Linux mmc core's `mrq->stop` model, so the controller
    has no automatic CMD12. CMD23 (SET_BLOCK_COUNT) also works, as an
    ordinary command, on cards whose SCR says they support it.

### 4.4 Data path (question 3): PIO through a 4 KiB FIFO, DMA deferred

- **The FIFO:**
  - one RAMB36 used as a 512 × 64-bit FIFO, i.e. 8 blocks;
  - first-word-fall-through: the head word sits in a register, so the MMIO
    read stays combinational like every other device;
  - **`SDM_DATA` (0x180):** a 64-bit load pops a word and a 64-bit store
    pushes one (byte order in §4.3);
  - `SDM_FIFO` gives the level;
  - **The FIFO is never reset implicitly by a command.** Software resets it
    with `SDM_FIFO[31]` before each data request. That lets a write driver
    pre-fill the FIFO before it sends CMD24/CMD25, and leaves the good
    blocks of a failed read in place to drain. CPU_RESETN and the
    native-mode load reset (§4.1) also empty it.
- **Pop-on-read is safe.** It is the same contract as `TRNG_DATA` and the
  UART's `RX_DATA`, because the pipeline never issues a load to MMIO
  speculatively (MMU_PLAN §7).
- **Only 64-bit accesses to `SDM_DATA` are defined.** For the record, the
  hardware pops or pushes one whole word on any access. A narrower load
  returns some of its bytes and discards the rest. A narrower store pushes a
  word whose other byte lanes are undefined. Drivers must not rely on
  either.
- **Underflow and overflow:** a pop from an empty FIFO returns 0 and does
  not move the pointer. A push into a full FIFO is dropped and sets the
  sticky **FIFO_ERR**. Neither happens to a driver that pops only when
  FIFO_THRESH is set, or checks `SDM_FIFO`.
- **CPU cost.**
  - An MMIO load takes the 2-cycle read path, and a store to cached DDR is
    a cache hit, so an unrolled copy loop should move 8 bytes in about
    4–6 cycles. That is roughly 130–200 MB/s of copy capability, so 25 MB/s
    costs ~15 % of the CPU.
  - This is an estimate. Phase 0 (§6) measures it on the board with the
    existing `SD_BUF` window, which has the same access path.
- **Why not DMA first.** A DMA master into DDR would be a fifth requester on
  `mem_read_write`'s arbiter, next to the cache, the blitter, core 2 and
  VGA. It would be non-coherent like the blitter, and the only cache
  maintenance today is a whole-cache walk (2048 lines, ≥ 20 µs plus dirty
  write-backs).
  - Linux's `arch_sync_dma_for_*` would then mean a full-cache walk per
    request, or a new address-range flush/invalidate in the cache. That
    range operation is worth having for its own sake, but it is a change to
    the cache FSM, which is on the critical timing path.
  - PIO already reaches the bus limit for a modest CPU share, needs no Linux
    DMA support, and touches nothing timing-critical.
  - Reserved for DMA: `SDM_DMA_ADDR` (0x150) and `SDM_CFG[16]` = DMA enable,
    which read 0 and are ignored on this design. Phase 4 adds them if the CPU
    share turns out to matter.
- **Double buffering.** It comes free with the 8-block FIFO and the clock
  stopping: the card fills block n+1 while the CPU drains block n.

### 4.5 Interrupts (question 3)

All four CPU interrupt sources are taken: timer, blitter, Ethernet, VGA.

- **Not a fifth source.** That would widen `INT_MASK`, which is saved in the
  interrupt frame (bits [42:39]). Every frame that software builds (xv6,
  Linux, Zephyr user-mode entry) would need a new bit, and an old-style
  frame would silently mask the SD interrupt. It also touches the
  pipeline's interrupt path.
- **Instead, a cascade on source 2**, which Ethernet uses today:
  - `INT_PENDING[2]` = (Ethernet & `INT2_EN[0]`) | (SD & `INT2_EN[1]`);
  - new registers in the IRQ block: `INT2_EN` (0xF00F_0050) and
    `INT2_PENDING` (0xF00F_0058), both reading 0 on old bitstreams;
  - **`INT2_EN` resets to 0b01**, Ethernet only. Old software sees source 2
    exactly as before.
  - Linux models source 2 as a chained irqchip with two children. Zephyr
    keeps polling SD, or does the same.
- **The SD interrupt itself** is a level:
  `|(SDM_STATUS & SDM_IRQ_EN)` over CMD_DONE, DATA_DONE, the error bits,
  and FIFO_THRESH. The handler clears the sticky bits with a
  write-1-to-clear to `SDM_STATUS`, the same contract as the blitter.
- **FIFO_THRESH** is a **live** level, independent of CMD_DONE and
  DATA_DONE, so a threaded handler can pace a multi-block transfer block by
  block. It is not cleared by writing to it; it drops when the condition
  stops holding.
  - **Read:** the FIFO holds at least one whole block of unread data
    (level ≥ block size / 8 words). It also stays set after the transfer has
    ended, until the FIFO is drained below one block. So after DATA_DONE the
    handler keeps popping until FIFO_THRESH drops; with whole blocks that
    leaves the FIFO empty.
  - **Write:** a write data phase is running, the FIFO has room for at
    least one more block, and fewer blocks have been pushed than the block
    count (or the count is open-ended). It is never set while idle, so an
    empty FIFO doesn't leave the interrupt stuck high.

### 4.6 Clocking and I/O timing (question 2)

- **CLK** comes from the fabric divider through an ODDR, as a forwarded
  clock. All flip-flops run on the 100 MHz `i_Clk`, so there is no new
  clock domain.
- **Default speed (≤ 25 MHz):**
  - the host drives CMD and DAT on the falling edge of CLK and samples on
    the rising edge, with ≥ 10 ns of margin each way at 10 ns granularity;
  - no timing risk.
- **High speed (50 MHz, after CMD6):**
  - the host drives right after the rising edge;
  - card output is valid ≤ 14 ns after the rising edge at the card and held
    ≥ 2.5 ns after the next one;
  - with the output and input pad delays, the valid window at the host's
    input flip-flop is roughly 21–29 ns after the launching fabric edge;
  - fabric edges come at 20 and 30 ns, so neither is safely inside that
    window;
  - so the inputs are sampled with an IDDR (5 ns steps), and the sample
    phase comes from `SDM_CFG[7:4]`. The driver picks the phase at the
    centre of the passing range by reading the SCR or CMD6 status at each
    phase and checking the CRC;
  - this is the plan's one real timing risk, and it is confined to
    high-speed mode.
- **Fallbacks for high speed:** IDELAYE2, which needs an IDELAYCTRL in bank
  35, or stopping at 25 MHz.

### 4.7 Registers (all 64-bit, device `0xF000_0000`)

| Offset | Name | RW | Content |
|---|---|---|---|
| 0x000 | `SD_CTRL` | RW | as today, plus `[18]` SD_MODE |
| 0x008 | `SD_DATA` | RW | SPI only, as today |
| 0x010 | `SD_STATUS` | R | as today (SPI busy, card present) |
| 0x100 | `SDM_CMD` | W | starts a command (§4.2) |
| 0x108 | `SDM_ARG` | RW | command argument |
| 0x110 | `SDM_RESP0` | R | response [63:0] (48-bit: [31:0] = status/OCR/RCA) |
| 0x118 | `SDM_RESP1` | R | R2 response [127:64] |
| 0x120 | `SDM_BLK` | RW | `[11:0]` block size (multiple of 8, 8–2048; `[2:0]` read 0), `[31:16]` block count (0 = open-ended) |
| 0x128 | `SDM_STATUS` | R/W1C | **live** (read-only): `[0]` CMD_BUSY, `[1]` DATA_BUSY, `[2]` DAT0 level, `[3]` CARD_PRESENT, `[4]` FIFO_THRESH (§4.5). **sticky** (write-1-to-clear): `[8]` CMD_DONE, `[9]` DATA_DONE, `[16]` CMD_TIMEOUT, `[17]` CMD_CRC_ERR, `[18]` CMD_END_ERR, `[19]` CMD_IDX_ERR, `[20]` DATA_TIMEOUT (no read start bit), `[21]` DATA_CRC_ERR, `[22]` DATA_END_ERR, `[23]` WRITE_CRC_ERR (token ≠ 010), `[24]` WRITE_NO_TOKEN, `[25]` BUSY_TIMEOUT (R1b or write busy), `[26]` FIFO_ERR (push to a full FIFO). `[31]` ERR_ANY (live) = OR of `[26:16]` |
| 0x130 | `SDM_IRQ_EN` | RW | interrupt enables, same bit positions as `SDM_STATUS` (`[4]`, `[9:8]`, `[26:16]`); others ignored |
| 0x138 | `SDM_TIMEOUT` | RW | `[31:0]` data timeout (read start bit), `[63:32]` busy timeout (R1b / write busy), both in CLK cycles. Reset 2 500 000 / 12 500 000 (100 ms / 500 ms at 25 MHz); software rescales for other clocks |
| 0x140 | `SDM_CFG` | RW | `[0]` 4-bit, `[2]` CLK_ON (free-running clock). Reserved until phase 3, reading 0: `[1]` high-speed edge, `[7:4]` HS sample phase. `[16]` DMA enable, reserved for phase 4. Reset 0 |
| 0x148 | `SDM_FIFO` | R/W | `[9:0]` level in words; writing `[31]` resets the FIFO |
| 0x150 | `SDM_DMA_ADDR` | — | reserved for phase 4 (reads 0) |
| 0x158 | `SDM_BLKDONE` | R | `[15:0]` good blocks in the current data command (§4.3); reset by an `SDM_CMD` with the data flag |
| 0x180 | `SDM_DATA` | RW | FIFO port: load pops, store pushes (64-bit) |
| 0x1F8 | `SDM_CAPS` | R | `[7:0]` version (1), `[8]` 4-bit, `[9]` high speed (0 until phase 3), `[23:16]` FIFO size in 512-byte blocks (8) |
| 0x200–0x3F8 | `SD_BUF` | RW | scratch RAM, as today |

Registers elsewhere:
- `CPU_FEATURES[21]` (0xF00F_0048) = 1: this controller is present.
- `INT2_EN` (0xF00F_0050, RW): `[0]` Ethernet, `[1]` SD, reset `0b01`.
- `INT2_PENDING` (0xF00F_0058, R): `[0]` the raw Ethernet interrupt, `[1]`
  the raw SD interrupt, both before `INT2_EN`.
- `INT_PENDING[2]` = |(`INT2_PENDING` & `INT2_EN`).

The SD-mode registers are writable at any time. They act only when
SD_MODE = 1, and writing `SDM_CMD` while CMD_BUSY is set is ignored. The
`addr[9]` decode of `SD_BUF` doesn't change, so 0x100–0x1F8 is free.

## 5. Software interface: what the runtime session reviews

- The mmc-core mapping for a Linux host driver (`drivers/mmc/host/klausscpu-sd.c`):
  - `.request`: fill `SDM_ARG`/`SDM_BLK`/`SDM_CFG`, write `SDM_CMD`, then
    move the data through `SDM_DATA` on FIFO_THRESH and finish on
    DATA_DONE (sending `mrq->stop` as CMD12);
  - `.set_ios`: `clk_div`, the bus width, the high-speed edge and phase, and
    power through `pwr_en`;
  - `.get_cd`: CARD_PRESENT.
  - Capabilities: `MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED |
    MMC_CAP_WAIT_WHILE_BUSY`, plus `MMC_CAP_CMD23` if wanted. The maximum
    block count is 65535 per request, or unlimited open-ended.
- An SDHCI-compatible register file was considered so that Linux's stock
  `sdhci` driver would work. It was rejected: the register file (≈ 256 bytes
  of fields, ADMA descriptors, presets) costs more fabric than the engine,
  and a custom host driver is about 500 lines over the mmc core. Say if
  you'd rather have it.
- Zephyr and bare metal keep SPI. A Zephyr `sdhc` driver for SD mode is
  optional later.

## 6. Phases

0. **Measure (software only).** On the board, time an unrolled 64-bit copy
   from `SD_BUF` to DDR, to confirm the PIO cost in §4.4. Then the runtime
   session's CMD18/CMD25 SPI port lands, giving a new baseline.
1. **RTL with default speed, 1-bit and 4-bit, PIO, interrupt cascade,
   `CPU_FEATURES[21]`.**
   - Unit testbench: a behavioural SD card model with CMD, R1/R2/R3/R6/R7,
     single-block and multi-block read and write, CRC errors, busy, and
     clock stopping.
   - Then `tools/vsim` SoC flows: SPI behaviour bit-identical to the master
     baseline (`socdiff`), and SD mode against the card model.
2. **Board test.** A bare-metal `test_sdnative`: power cycle, identification,
   4-bit, CMD17/18/24/25, compared against SPI-mode reads of the same
   blocks. Then the Linux driver (runtime session). Measure throughput.
3. **High speed at 50 MHz:** IDDR sampling with a phase register and
   calibration, CMD6 switch.
4. **Optional DMA:** a fifth DDR master, and address-range cache
   flush/invalidate (also useful to Linux as a whole). Only if the CPU share
   in phases 2 and 3 matters.

## 7. Cost and timing (question 6)

| Part | LUTs | FFs | BRAM |
|---|---|---|---|
| Command engine (frame, CRC7, 136-bit response, FSM) | ~300 | ~250 | – |
| Data engine (4× CRC16, nibble/byte packer, counters, busy, token) | ~400 | ~300 | – |
| Registers, clock divider and gating, interrupt, cascade | ~200 | ~250 | – |
| FIFO (FWFT around one RAMB36) | ~80 | ~80 | 1 |
| IOBUF muxing, ODDR, IDDR | ~30 | – | – |
| **Total** | **~1.0–1.2k (≈ 2 %)** | **~0.9k** | **1** |

Today, on d9cf6c2 LEAN: 34.0k LUTs (54 %), 10.9k slices (69 %), 74.5 BRAM
(55 %), 146 bonded IOBs (no new pins needed).

Timing risk is **low** for phases 1–2:
- The block sits behind `KEEP_HIERARCHY` on the MMIO bus, whose read return
  is already registered in `bus_splitter`.
- It has no path into the core except one more leaf on the MMIO read mux and
  one OR term on interrupt source 2, which is already a registered level
  (`r_eth_irq`).
- The 69 % slice utilisation slightly raises congestion; placement noise is
  about ±0.3 ns.

Phase 3's risk is I/O timing (§4.6), not fabric timing. Phase 4 touches the
cache arbiter and is the only phase with real core-timing risk.
