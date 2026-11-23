#!/usr/bin/env python3
"""Sensor faults on GDN's inputs in SITL (roadmap R2.6b; agreed: on GDN's inputs, not on
the simulated drivers). tools/sim_adapter.py applies them to what it publishes for
eskf_node, and to PX4's IMU through its relay.

A fault is one sensor, one kind, a start and a length, counted from the route's start
(/gdn/sim/route_start, as the GNSS cut):

    sensor   agl | baro | mag | imu      (GDN's altimeter, barometer, magnetometer, IMU)
    kind     dropout                     nothing reaches GDN
             stuck                       the last value before the fault, repeated
             noise                       Gaussian noise added, sigma in the sensor's units
                                         (agl, baro m; mag uT; imu: accelerometer m/s^2,
                                         and optionally the gyro rad/s)

As one string (GDN_FAULTS, the adapter's "faults" parameter), faults separated by ";":

    sensor:kind:at_s:duration_s[:sigma[:sigma_gyro]]
    agl:dropout:30:20;baro:stuck:90:30;mag:noise:150:20:40;imu:noise:200:10:2:0.05

    python3 tools/sim_faults.py SPEC         # parse and print
    python3 tools/sim_faults.py --selfcheck  # the fault logic on made-up data
"""
import sys
from dataclasses import dataclass

SENSORS = ("agl", "baro", "mag", "imu")
KINDS = ("dropout", "stuck", "noise")


@dataclass(frozen=True)
class Fault:
    sensor: str
    kind: str
    at_s: float
    duration_s: float
    sigma: float = 0.0
    sigma_gyro: float = 0.0

    def active(self, t_route):
        return t_route is not None and self.at_s <= t_route < self.at_s + self.duration_s

    def text(self):
        s = f"{self.sensor} {self.kind} at {self.at_s:g} s for {self.duration_s:g} s"
        if self.kind == "noise":
            s += f", sigma {self.sigma:g}" + (f" (gyro {self.sigma_gyro:g} rad/s)" if self.sigma_gyro else "")
        return s

    def spec(self):
        f = [self.sensor, self.kind, f"{self.at_s:g}", f"{self.duration_s:g}"]
        if self.kind == "noise":
            f.append(f"{self.sigma:g}")
            if self.sigma_gyro:
                f.append(f"{self.sigma_gyro:g}")
        return ":".join(f)


def parse(spec):
    """Faults from the string form; ValueError naming the bad one."""
    out = []
    for item in (spec or "").replace(" ", "").split(";"):
        if not item:
            continue
        p = item.split(":")
        if len(p) < 4 or p[0] not in SENSORS or p[1] not in KINDS:
            raise ValueError(f"fault {item!r}: sensor:kind:at_s:duration_s[:sigma[:sigma_gyro]], "
                             f"sensor {'|'.join(SENSORS)}, kind {'|'.join(KINDS)}")
        try:
            v = [float(x) for x in p[2:]]
        except ValueError as e:
            raise ValueError(f"fault {item!r}: numbers expected after the kind") from e
        at, dur = v[0], v[1]
        sigma = v[2] if len(v) > 2 else 0.0
        sg = v[3] if len(v) > 3 else 0.0
        if at < 0 or dur <= 0 or sigma < 0 or sg < 0:
            raise ValueError(f"fault {item!r}: at_s >= 0, duration_s > 0, sigmas >= 0")
        if p[1] == "noise" and sigma <= 0 and sg <= 0:
            raise ValueError(f"fault {item!r}: noise needs a sigma")
        if p[1] != "noise" and len(v) > 2:
            raise ValueError(f"fault {item!r}: only noise takes a sigma")
        if sg and p[0] != "imu":
            raise ValueError(f"fault {item!r}: only the imu has a gyro sigma")
        out.append(Fault(p[0], p[1], at, dur, sigma, sg))
    for s in SENSORS:     # one at a time per sensor
        fs = sorted((f for f in out if f.sensor == s), key=lambda f: f.at_s)
        for a, b in zip(fs, fs[1:]):
            if b.at_s < a.at_s + a.duration_s:
                raise ValueError(f"faults {a.spec()} and {b.spec()} overlap on {s}")
    return out


class Injector:
    """Applies the faults to one sensor's values. apply(t_route, value) returns the value
    to publish (a float or a tuple of floats) or None for "publish nothing"; for the IMU,
    value is (accel xyz, gyro xyz)."""

    def __init__(self, faults, sensor, rng):
        self.faults = [f for f in faults if f.sensor == sensor]
        self.rng = rng
        self.last = None              # the last good value, for "stuck"
        self.frozen = None
        self.current = None           # the fault in force, for on/off events

    def fault_at(self, t_route):
        return next((f for f in self.faults if f.active(t_route)), None)

    def apply(self, t_route, value):
        f = self.fault_at(t_route)
        changed = f is not self.current
        self.current = f
        if f is None:
            self.frozen = None
            self.last = value
            return value, changed
        if f.kind == "dropout":
            return None, changed
        if f.kind == "stuck":
            if self.frozen is None:
                self.frozen = self.last if self.last is not None else value
            return self.frozen, changed
        # noise
        if isinstance(value, tuple) and len(value) == 2 and isinstance(value[0], tuple):   # the IMU
            acc = tuple(x + self.rng.normal(0.0, f.sigma) if f.sigma else x for x in value[0])
            gyr = tuple(x + self.rng.normal(0.0, f.sigma_gyro) if f.sigma_gyro else x for x in value[1])
            return (acc, gyr), changed
        if isinstance(value, tuple):
            return tuple(x + self.rng.normal(0.0, f.sigma) for x in value), changed
        return value + self.rng.normal(0.0, f.sigma), changed


def selfcheck():
    import numpy as np
    ok = True

    def ck(c, what):
        nonlocal ok
        print(f"{'PASS' if c else 'FAIL'}  selfcheck: {what}")
        ok &= bool(c)

    fs = parse("agl:dropout:30:20;baro:stuck:90:30;mag:noise:150:20:40;imu:noise:200:10:2:0.05")
    ck([f.spec() for f in fs] == ["agl:dropout:30:20", "baro:stuck:90:30", "mag:noise:150:20:40",
                                  "imu:noise:200:10:2:0.05"], "the string form round-trips")
    for bad in ("agl:melt:1:2", "agl:dropout:1", "agl:noise:1:2", "agl:stuck:1:2:3", "mag:noise:1:2:3:4",
                "baro:dropout:10:20;baro:stuck:25:5"):
        try:
            parse(bad)
            ck(False, f"{bad!r} refused")
        except ValueError:
            ck(True, f"{bad!r} refused")
    rng = np.random.default_rng(1)
    inj = Injector(fs, "baro", rng)
    out = [inj.apply(t, float(t))[0] for t in np.arange(80.0, 130.0, 0.5)]
    stuck = [v for t, v in zip(np.arange(80.0, 130.0, 0.5), out) if 90 <= t < 120]
    ck(all(v == 89.5 for v in stuck) and out[-1] == 129.5, "stuck repeats the last value before it, then lets go")
    inj = Injector(fs, "agl", rng)
    out = [inj.apply(t, 1.0)[0] for t in np.arange(0.0, 60.0, 0.5)]
    ck(sum(v is None for v in out) == 40 and out[0] == 1.0 and out[-1] == 1.0, "dropout publishes nothing for 20 s")
    inj = Injector(fs, "mag", rng)
    vals = np.array([inj.apply(150 + 0.01 * k, (20.0, 0.0, -40.0))[0] for k in range(2000)])
    ck(abs(vals[:, 0].std() - 40) < 3 and abs(vals[:, 2].mean() + 40) < 3, "noise has the sigma asked, no bias")
    inj = Injector(fs, "imu", rng)
    v = [inj.apply(205.0, ((0.0, 0.0, -9.8), (0.0, 0.0, 0.0)))[0] for _ in range(4000)]
    acc = np.array([a for a, _ in v])
    gyr = np.array([g for _, g in v])
    ck(abs(acc[:, 0].std() - 2) < 0.15 and abs(gyr[:, 1].std() - 0.05) < 0.004, "imu noise: accel and gyro sigmas")
    print("SELFCHECK PASSED" if ok else "SELFCHECK FAILED")
    return ok


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--selfcheck":
        sys.exit(0 if selfcheck() else 1)
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    try:
        for f in parse(sys.argv[1]):
            print(f.text())
    except ValueError as e:
        sys.exit(f"ERROR: {e}")


if __name__ == "__main__":
    main()
