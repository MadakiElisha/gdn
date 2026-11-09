#!/usr/bin/env python3
"""Record a static H30WP log straight from the serial port, for the R1.6 Allan fit.

    python3 tools/h30wp_check.py                            # first: is it sending? (docs/hardware/H30WP_SETUP.md)
    systemd-inhibit --what=sleep:idle python3 tools/h30wp_log.py --hours 2.25 --out log/h30wp_static
    python3 tools/h30wp_log.py --finalize log/h30wp_static   # rebuild allan.csv from raw.csv
    python3 tools/allan_fit.py --csv log/h30wp_static/allan.csv --skip 900 --name h30wp \\
        --out config/imu/h30wp.yaml --plot log/h30wp_static/allan.png

Why not `ros2 bag record` of tools/h30wp_node.py: the H30WP's default output has no
sample timestamp (DATA ID 0x51), so the node stamps messages on arrival, and USB
delivers frames in bursts, milliseconds apart. That jitter lands in the Allan deviation.
Here the time of frame n is n / rate instead, from the IMU's own frame numbers, with
the rate fitted to host time over the whole log (or the IMU timestamps when it sends
them). Reads only: nothing is sent to the IMU. Decoding is tools/h30wp_check.py's.

Output, in --out DIR:
  raw.csv    one row per frame as received: host time, frame number, IMU timestamp (if
             sent), accel m/s^2, gyro rad/s, temperature (if sent). Written as it goes,
             so an interrupted log keeps everything up to that point.
  allan.csv  t (us), ax..gz, for tools/allan_fit.py --csv. Lost frames (gaps in the
             frame number) keep their slot on the time grid, and allan_fit reports them.
Axes are the IMU's own front-left-up, as printed on the lid.
"""
import argparse
import csv
import math
import os
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from h30wp_check import Parser, find_ports  # noqa: E402

DEG = math.pi / 180.0
WRAP = 60000   # frame numbers run 1..60000


def open_port(path, baud):
    """Raw, read-only, non-blocking; termios only, so no pyserial needed."""
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = getattr(termios, f"B{baud}")
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def read_for(fd, parser, seconds, on_frames):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            chunk = os.read(fd, 65536)
        except BlockingIOError:
            chunk = b""
        if chunk:
            on_frames(time.time(), parser.feed(chunk))
        else:
            time.sleep(0.002)


def log(args):
    ports = find_ports()
    path = args.port or (ports[-1] if ports else None)
    if not path:
        sys.exit("no /dev/ttyUSB* or /dev/ttyACM*: is the converter plugged in? (tools/h30wp_check.py)")
    os.makedirs(args.out, exist_ok=True)
    raw_path = os.path.join(args.out, "raw.csv")
    fd = open_port(path, args.baud)
    p = Parser()
    st = {"n": 0, "lost": 0, "last": None, "sum": [0.0] * 6, "k": 0, "temp": float("nan"), "t0": time.time()}
    with open(raw_path, "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(["host_s", "tid", "imu_ts_us", "ax", "ay", "az", "gx", "gy", "gz", "temp_c"])

        def on_frames(t_host, frames):
            for tid, d in frames:
                if 0x10 not in d or 0x20 not in d:
                    continue
                acc, gyr = d[0x10][:3], [v * DEG for v in d[0x20][:3]]
                if st["last"] is not None:
                    st["lost"] += (tid - st["last"] - 1) % WRAP
                st["last"] = tid
                st["n"] += 1
                temp = d.get(0x01, float("nan"))
                if 0x01 in d:
                    st["temp"] = temp
                st["sum"] = [s + x for s, x in zip(st["sum"], acc + gyr)]
                st["k"] += 1
                w.writerow([f"{t_host:.6f}", tid, d.get(0x51, ""), *(f"{x:.6f}" for x in acc),
                            *(f"{x:.9f}" for x in gyr), f"{temp:.2f}"])

        total = args.hours * 3600.0
        print(f"logging {path} at {args.baud} baud for {args.hours:g} h into {raw_path} (Ctrl+C stops early; "
              "keep the IMU still)", flush=True)
        try:
            while total - (time.time() - st["t0"]) > 0.5:
                read_for(fd, p, min(60.0, total - (time.time() - st["t0"])), on_frames)
                fo.flush()
                m = [s / max(st["k"], 1) for s in st["sum"]]
                el = time.time() - st["t0"]
                print(f"{time.strftime('%H:%M:%S')}  {el / 60:6.1f} of {total / 60:.0f} min  frames {st['n']}  "
                      f"lost {st['lost']}  bad {p.bad}  |a| {math.hypot(*m[:3]):.4f} m/s^2  "
                      f"gyro {math.hypot(*m[3:]) / DEG:.4f} deg/s  temp {st['temp']:.2f} C", flush=True)
                st["sum"], st["k"] = [0.0] * 6, 0
        except KeyboardInterrupt:
            print("stopped early")
    os.close(fd)
    finalize(args.out)


def finalize(out_dir):
    raw_path = os.path.join(out_dir, "raw.csv")
    rows = list(csv.reader(open(raw_path)))[1:]
    if len(rows) < 100:
        sys.exit(f"{raw_path}: only {len(rows)} frames")
    # Frame index from the frame numbers (wrapping), so lost frames keep their slot.
    idx, n, last = [], 0, None
    for r in rows:
        tid = int(r[1])
        n += 0 if last is None else (tid - last) % WRAP
        last = tid
        idx.append(n)
    ts = [r[2] for r in rows]
    if all(ts):
        # IMU timestamps (uint32 us, wraps after about 71.6 minutes): unwrap them.
        t_us, off, prev = [], 0, None
        for v in (int(x) for x in ts):
            if prev is not None and v < prev:
                off += 1 << 32
            t_us.append(v + off - int(ts[0]))
            prev = v
        how = "IMU sample timestamps"
        rate = (len(idx) - 1) / (t_us[-1] * 1e-6) if t_us[-1] > 0 else float("nan")
    else:
        # Least-squares line through (frame index, host time): the slope is the period.
        # The residual is the two clocks drifting apart (a few ppm), not lost frames.
        hs = [float(r[0]) for r in rows]
        mi, mh = sum(idx) / len(idx), sum(hs) / len(hs)
        sxx = sum((i - mi) ** 2 for i in idx)
        period = sum((i - mi) * (h - mh) for i, h in zip(idx, hs)) / sxx
        rate = 1.0 / period
        resid = max(abs(h - (mh + (i - mi) * period)) for i, h in zip(idx, hs))
        t_us = [i * period * 1e6 for i in idx]
        how = f"frame numbers, rate fitted to host time (worst host-time residual {resid * 1e3:.1f} ms)"
    with open(os.path.join(out_dir, "allan.csv"), "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(["t", "ax", "ay", "az", "gx", "gy", "gz"])
        for tu, r in zip(t_us, rows):
            w.writerow([f"{tu:.0f}", *r[3:9]])
    lost = idx[-1] + 1 - len(idx)
    hours = t_us[-1] / 3.6e9
    print(f"allan.csv: {len(rows)} frames, {hours:.2f} h, rate {rate:.4f} Hz, {lost} lost frames; timing: {how}")
    if hours < 2.0:
        print("WARNING: shorter than 2 h: the drift part of the fit will be weak (config/imu/README.md)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device (default: the last /dev/ttyUSB* or /dev/ttyACM*)")
    ap.add_argument("--baud", type=int, default=460800, help="default 460800 (the H30's factory setting)")
    ap.add_argument("--hours", type=float, default=2.25, help="log length (default 2.25: 2 h plus 15 min warm-up)")
    ap.add_argument("--out", default="log/h30wp_static", help="output directory")
    ap.add_argument("--finalize", metavar="DIR", help="only rebuild DIR/allan.csv from DIR/raw.csv")
    a = ap.parse_args()
    if a.finalize:
        finalize(a.finalize)
    else:
        log(a)
