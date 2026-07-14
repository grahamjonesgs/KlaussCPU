#!/usr/bin/env python3
# parse_haz.py — decode the M7 hazard-probe UART stream into a per-kernel table.
#
# Each probe line is 11 fixed 16-hex fields (self-delimiting), in order:
#   id cycles instr STALL_DATA STALL_LOADUSE STALL_FLAGS STALL_SP
#   STALL_MULDIV BRANCH_FLUSH IF_MISS MEM_WAIT
# (0xF00D_0008,0010,00B0,00B8,00C0,00C8,00D0,00D8,00E0,00E8)
#
# usage:  parse_haz.py <uart_capture_file>   [--csv out.csv]
# Reads stdin if no file given.  Ignores any non-176-hex lines (banners etc.).

import sys, re

NAMES = {0: "alu", 1: "mem_stream", 2: "ptr_chase", 3: "branchy",
         4: "calls_fib", 5: "muldiv"}
FIELDS = ["id", "cycles", "instr", "data", "loaduse", "flags", "sp",
          "muldiv", "brflush", "ifmiss", "memwait"]
HAZ = ["data", "loaduse", "flags", "sp", "muldiv", "brflush", "ifmiss", "memwait"]

def parse(text):
    rows = []
    for ln in text.splitlines():
        s = ln.strip()
        if len(s) == 176 and re.fullmatch(r"[0-9A-Fa-f]+", s):
            vals = [int(s[i:i+16], 16) for i in range(0, 176, 16)]
            rows.append(dict(zip(FIELDS, vals)))
    return rows

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    csv_out = None
    if "--csv" in sys.argv:
        csv_out = sys.argv[sys.argv.index("--csv") + 1]
    text = open(args[0]).read() if args else sys.stdin.read()
    rows = parse(text)
    if not rows:
        print("no probe lines found (expected 176-hex-char lines)", file=sys.stderr)
        sys.exit(1)

    hdr = f"{'kernel':<11} {'cycles':>12} {'instr':>12} {'CPI':>6} | " + \
          " ".join(f"{h:>8}" for h in HAZ)
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        name = NAMES.get(r["id"], f"?{r['id']}")
        cyc, ins = r["cycles"], r["instr"]
        cpi = cyc / ins if ins else 0.0
        cells = []
        for h in HAZ:
            pct = (100.0 * r[h] / cyc) if cyc else 0.0
            cells.append(f"{pct:7.2f}%")
        print(f"{name:<11} {cyc:>12} {ins:>12} {cpi:>6.3f} | " + " ".join(cells))

    # secondary view: hazard cycles as % of cycles is above; also print raw counts
    print("\nraw hazard cycle counts:")
    hdr2 = f"{'kernel':<11} " + " ".join(f"{h:>12}" for h in HAZ)
    print(hdr2)
    for r in rows:
        name = NAMES.get(r["id"], f"?{r['id']}")
        print(f"{name:<11} " + " ".join(f"{r[h]:>12}" for h in HAZ))

    if csv_out:
        with open(csv_out, "w") as f:
            f.write("kernel," + ",".join(FIELDS[1:]) + ",cpi\n")
            for r in rows:
                name = NAMES.get(r["id"], f"?{r['id']}")
                cpi = r["cycles"] / r["instr"] if r["instr"] else 0
                f.write(name + "," + ",".join(str(r[k]) for k in FIELDS[1:]) +
                        f",{cpi:.4f}\n")
        print(f"\nwrote {csv_out}", file=sys.stderr)

if __name__ == "__main__":
    main()
