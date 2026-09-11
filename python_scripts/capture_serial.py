#!/usr/bin/env python3
"""Capture a run straight from the Teensy into a finished CSV.

Replaces select-all-in-the-Serial-Monitor / paste-into-a-spreadsheet.  It
watches the serial port, records everything between

    ===== DATA START =====
    ===== DATA END =====

and writes a file in exactly the layout the spreadsheet was producing -- title
row, header row, data rows, plus the derived columns (Efficiency, ABS LC1) that
were being added by hand.  Bowden Wrap-Angle Studio opens the result directly.

    python scripts/capture_serial.py --label "110 deg"
    python scripts/capture_serial.py --list          # show serial ports
    python scripts/capture_serial.py --replay run.txt --label "110 deg"

It REPLACES the Arduino Serial Monitor -- only one program can hold a serial
port at a time, so close the monitor before starting this.  Everything the
firmware prints is echoed, including the pre-tension readout, and the script
stays open across runs: each START/STOP pair becomes its own CSV.

Needs pyserial for live capture (`pip install pyserial`); --replay works
without it, for re-processing text already pasted into a file.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import re
import statistics
import sys

START_MARK = "===== DATA START ====="
END_MARK = "===== DATA END ====="
# The firmware can print the millis() value of the LED rising edge; if it does,
# it is carried into the output as an exact video-sync anchor.
LED_RE = re.compile(r"#\s*LED_ON_MS\s*,\s*(\d+)")


def parse_stream(lines):
    """Pull one run out of a stream of serial lines.

    Returns (header, rows, led_on_ms, notes).  Tolerates the chatter the
    firmware prints around the data block, and stops at the END marker.
    """
    header, rows, led_on_ms, notes = None, [], None, []
    capturing = False
    for raw in lines:
        line = raw.rstrip("\r\n")
        if START_MARK in line:
            capturing, header, rows = True, None, []
            continue
        if END_MARK in line:
            break
        if not capturing:
            continue
        m = LED_RE.search(line)
        if m:
            led_on_ms = int(m.group(1))
            continue
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",")]
        if header is None:
            # the first line after the marker is the column header
            if len(parts) > 2 and not _is_number(parts[0]):
                header = parts
            continue
        if len(parts) == len(header):
            rows.append(parts)
        else:
            notes.append(f"skipped a malformed row ({len(parts)} of {len(header)} fields)")
    return header, rows, led_on_ms, notes


def _is_number(s):
    try:
        float(s)
        return True
    except Exception:
        return False


def derive(header, rows):
    """Add the columns that were being computed in the spreadsheet.

    Efficiency is min(|LC1|,|LC2|) / max(|LC1|,|LC2|) -- the same definition as
    before, so old and new files stay comparable.
    """
    try:
        i1 = header.index("LoadCell1_N")
        i2 = header.index("LoadCell2_N")
    except ValueError:
        return header, rows
    eff = []
    for r in rows:
        try:
            a, b = abs(float(r[i1])), abs(float(r[i2]))
            eff.append(min(a, b) / max(a, b) if max(a, b) > 0 else float("nan"))
        except Exception:
            eff.append(float("nan"))
    good = [e for e in eff if e == e]
    avg = statistics.fmean(good) if good else float("nan")
    sd = statistics.pstdev(good) if len(good) > 1 else 0.0
    out_h = header + ["Efficiency", "Avg eff", "Std eff", "Efficiency (%)", "ABS LC1 (N)"]
    out_r = []
    for r, e in zip(rows, eff):
        try:
            abs1 = f"{abs(float(r[i1])):.6f}"
        except Exception:
            abs1 = ""
        out_r.append(r + [f"{e:.10f}", f"{avg:.10f}", f"{sd:.11f}", f"{e * 100:.8f}", abs1])
    return out_h, out_r


def write_csv(path, label, header, rows, led_on_ms):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        title = ["Video", label] + [""] * max(0, len(header) - 2)
        w.writerow(title)
        if led_on_ms is not None:
            w.writerow([f"# LED_ON_MS={led_on_ms}"] + [""] * (len(header) - 1))
        w.writerow(header)
        w.writerows(rows)


def led_column(header):
    """Index of the LED-state column, if the log carries one."""
    for i, h in enumerate(header):
        k = h.lower()
        if "led" in k and "ms" not in k:
            return i
    return None


def led_edges(header, rows):
    """Rising edges of the LED column, in seconds from the first row.

    Reported at the bench so a run where the flashes fell outside the logging
    window is obvious immediately, rather than at analysis time when the video
    has already been filed away.
    """
    i = led_column(header)
    if i is None:
        return None, []
    try:
        ti = header.index("Timestamp_ms")
    except ValueError:
        return header[i], []
    vals, times = [], []
    for r in rows:
        try:
            vals.append(float(r[i]))
            times.append(float(r[ti]) / 1000.0)
        except Exception:
            vals.append(float("nan"))
            times.append(float("nan"))
    finite = [v for v in vals if v == v]
    if not finite or max(finite) == min(finite):
        return header[i], []
    mid = 0.5 * (max(finite) + min(finite))
    t0 = times[0]
    edges = []
    for j in range(1, len(vals)):
        if vals[j] == vals[j] and vals[j - 1] == vals[j - 1]:
            if vals[j] > mid >= vals[j - 1]:
                edges.append(times[j] - t0)
    return header[i], edges


def summarise(header, rows, led_on_ms):
    if not rows:
        return "no data rows captured"
    try:
        ti = header.index("Timestamp_ms")
        t0, t1 = float(rows[0][ti]), float(rows[-1][ti])
        span = (t1 - t0) / 1000.0
        rate = (len(rows) - 1) / span if span > 0 else float("nan")
        s = f"{len(rows)} rows over {span:.3f} s ({rate:.1f} Hz)"
        if led_on_ms is not None:
            s += f"; LED marker at {led_on_ms} ms, {(t0 - led_on_ms):.0f} ms before the first row"
        name, edges = led_edges(header, rows)
        if name is None:
            s += "\n  no LED column in this log — video sync will have to fall back to correlation"
        elif not edges:
            s += (f"\n  WARNING: '{name}' never goes high — the flashes were not captured. "
                  f"Check they fire after logging starts.")
        else:
            times = ", ".join(f"{e:.3f}" for e in edges[:6])
            s += (f"\n  LED '{name}': {len(edges)} flash(es) at t+{times} s"
                  + (" ..." if len(edges) > 6 else ""))
            if len(edges) >= 2:
                gaps = [edges[i + 1] - edges[i] for i in range(len(edges) - 1)]
                s += f"  (gaps {', '.join(f'{g:.3f}' for g in gaps[:5])} s)"
        return s
    except Exception:
        return f"{len(rows)} rows"


def live(port, baud, label, outdir, quiet=False):
    """Own the serial port for a whole session.

    This REPLACES the Arduino Serial Monitor -- only one program can hold a
    serial port, so the monitor must be closed.  Everything the firmware prints
    is echoed here (the pre-tension readout included, which is what you watch
    while getting LC1 into range), and each START/STOP pair is written out as
    its own CSV.  Stays open for the next run, so a whole angle sweep is one
    invocation.
    """
    try:
        import serial
    except ImportError:
        print("pyserial is not installed.  pip install pyserial", file=sys.stderr)
        return 2

    print(f"listening on {port} at {baud} — this replaces the Serial Monitor, "
          f"so keep that closed.")
    print("Press START to begin a run, STOP to end it. Ctrl-C when the session is done.\n")

    buf, capturing, rows_seen, n_runs = [], False, 0, 0
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            while True:
                line = ser.readline().decode("utf-8", "replace")
                if not line:
                    continue
                if capturing:
                    buf.append(line)

                if START_MARK in line:
                    capturing, buf, rows_seen = True, [line], 0
                    print("\n>>> recording ...")
                    continue

                if END_MARK in line and capturing:
                    capturing = False
                    n_runs += 1
                    print(f"\r    {rows_seen} rows — run complete")
                    run_label = label if n_runs == 1 else f"{label}_{n_runs}"
                    finish(buf, run_label, outdir)
                    print("\nready for the next run (Ctrl-C to finish the session)\n")
                    continue

                if capturing and line.count(",") > 5:
                    rows_seen += 1
                    if rows_seen % 25 == 0:
                        print(f"\r    {rows_seen} rows", end="", flush=True)
                elif not quiet:
                    # firmware chatter: the pre-tension readout lives here
                    t = line.rstrip()
                    if t:
                        # the row counter rewrites one line with \r; break out of
                        # it before echoing so the two do not collide
                        print(("\n" if capturing and rows_seen else "") + f"    {t}")
    except KeyboardInterrupt:
        print("\n\nsession ended.")
        if capturing and buf:
            print("a run was still in progress — saving what was captured")
            finish(buf, f"{label}_partial", outdir)
    except Exception as exc:
        print(f"\nserial error: {exc}", file=sys.stderr)
        return 1
    print(f"{n_runs} run(s) written to {os.path.abspath(outdir)}")
    return 0


def finish(lines, label, outdir):
    header, rows, led_on_ms, notes = parse_stream(lines)
    if not header or not rows:
        print("no complete run found (looked for the DATA START / DATA END markers)",
              file=sys.stderr)
        return 1
    header, rows = derive(header, rows)
    os.makedirs(outdir, exist_ok=True)
    safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in label) or "run"
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(outdir, f"{safe}_{stamp}.csv")
    write_csv(path, label, header, rows, led_on_ms)
    print(f"wrote {path}")
    print(f"  {summarise(header, rows, led_on_ms)}")
    for n in set(notes):
        print(f"  note: {n}")
    if led_on_ms is None:
        print("  no LED marker in this run -- add one to the firmware for exact video sync")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--label", default="run", help='trial label, e.g. "110 deg"')
    ap.add_argument("--outdir", default=".", help="where to write the CSV")
    ap.add_argument("--quiet", action="store_true",
                    help="do not echo the firmware's non-data output")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--replay", help="parse a text file of already-captured serial output")
    a = ap.parse_args()

    if a.list:
        try:
            from serial.tools import list_ports
        except ImportError:
            print("pyserial is not installed.  pip install pyserial", file=sys.stderr)
            return 2
        ports = list(list_ports.comports())
        if not ports:
            print("no serial ports found")
        for p in ports:
            print(f"  {p.device}  {p.description}")
        return 0

    if a.replay:
        with open(a.replay, errors="replace") as fh:
            return finish(fh.readlines(), a.label, a.outdir)

    port = a.port
    if not port:
        try:
            from serial.tools import list_ports
            cands = [p.device for p in list_ports.comports()
                     if "usb" in (p.description or "").lower()
                     or "teensy" in (p.description or "").lower()
                     or "acm" in p.device.lower()]
            if len(cands) == 1:
                port = cands[0]
                print(f"using {port}")
            else:
                print("specify --port (or --list to see them)", file=sys.stderr)
                return 2
        except ImportError:
            print("pyserial is not installed.  pip install pyserial", file=sys.stderr)
            return 2
    return live(port, a.baud, a.label, a.outdir, a.quiet)


if __name__ == "__main__":
    sys.exit(main())