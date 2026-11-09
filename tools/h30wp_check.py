#!/usr/bin/env python3
"""Check a Yesense / WHEELTEC H30WP IMU on a serial port, and say plainly whether it works.

    python3 tools/h30wp_check.py                    # finds the port, 460800 baud, 5 s
    python3 tools/h30wp_check.py --port /dev/ttyUSB0 --seconds 10
    python3 tools/h30wp_check.py --scan             # try the usual baud rates
    python3 tools/h30wp_check.py --udev             # print a udev rule for a fixed name

Frames follow the YESENSE Communication Protocol (v2.3): header 59 53, frame number
(2 bytes), payload length (1 byte), payload of DATA ID / LEN / DATA blocks, two
checksum bytes over everything from the frame number to the end of the payload.
Values are little-endian signed integers: acceleration x 1e-6 m/s^2, angular rate
x 1e-6 deg/s, raw magnetic field x 1e-3 mGauss, Euler angles x 1e-6 deg,
temperature x 0.01 C. Axes are the IMU's own front-left-up (FLU), as printed on
the housing. GDN converts to front-right-down later; this script doesn't.

Needs pyserial (sudo apt install python3-serial). Leave the IMU still while it runs.
"""
import argparse
import glob
import os
import struct
import sys
import time

HEADER = b"\x59\x53"
SCALE = {0x10: 1e-6, 0x20: 1e-6, 0x30: 1e-6, 0x31: 1e-3, 0x40: 1e-6, 0x41: 1e-6}
NAMES = {0x01: "temperature", 0x10: "acceleration", 0x20: "angular rate", 0x30: "mag (normalised)",
         0x31: "mag (raw)", 0x40: "euler angles", 0x41: "quaternion", 0x50: "utc time",
         0x51: "sample timestamp", 0x52: "dataready timestamp", 0x68: "position",
         0x70: "velocity", 0x80: "nav status"}

# The example frame from section 3.1.1 of the protocol document, used as a self-test.
EXAMPLE = bytes([
    0x59, 0x53, 0x40, 0x92, 0x58, 0x10, 0x0c, 0x3b, 0x21, 0xfe, 0xff, 0x89, 0x2c, 0xfe, 0xff,
    0x59, 0x9c, 0x6a, 0xff, 0x20, 0x0c, 0xd1, 0xa2, 0x02, 0x00, 0xfa, 0xb1, 0x05, 0x00, 0x38,
    0xd0, 0x00, 0x00, 0x30, 0x0c, 0x20, 0xff, 0xf1, 0x07, 0xd0, 0xa0, 0xea, 0xf4, 0xc0, 0x8e,
    0xc6, 0xef, 0x31, 0x0c, 0xb4, 0x08, 0x02, 0x00, 0xa2, 0x29, 0xfd, 0xff, 0xb8, 0xd8, 0xfb,
    0xff, 0x40, 0x0c, 0xcb, 0xe4, 0xf4, 0xff, 0xb1, 0xbe, 0x09, 0x00, 0x30, 0x32, 0xb6, 0xf6,
    0x41, 0x10, 0xd4, 0x31, 0x03, 0x00, 0x4d, 0xec, 0xff, 0xff, 0x86, 0xe5, 0xff, 0xff, 0xa9,
    0x14, 0xf1, 0xff, 0x12, 0xf3])


def checksum(buf):
    ck1 = ck2 = 0
    for b in buf:
        ck1 = (ck1 + b) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    return ck1, ck2


def parse_payload(payload):
    out, i = {}, 0
    while i + 2 <= len(payload):
        did, ln = payload[i], payload[i + 1]
        data = payload[i + 2:i + 2 + ln]
        if len(data) < ln:
            break
        if did == 0x01 and ln == 2:
            out[did] = struct.unpack("<h", data)[0] * 0.01
        elif did in (0x51, 0x52) and ln == 4:
            out[did] = struct.unpack("<I", data)[0]
        elif did in SCALE and ln % 4 == 0:
            out[did] = [v * SCALE[did] for v in struct.unpack(f"<{ln // 4}i", data)]
        else:
            out[did] = data
        i += 2 + ln
    return out


class Parser:
    """Pulls checked frames out of a byte stream."""
    def __init__(self):
        self.buf = bytearray()
        self.good = self.bad = 0

    def feed(self, chunk):
        self.buf += chunk
        frames = []
        while True:
            k = self.buf.find(HEADER)
            if k < 0:
                del self.buf[:-1]
                return frames
            del self.buf[:k]
            if len(self.buf) < 5:
                return frames
            n = self.buf[4]
            if len(self.buf) < 5 + n + 2:
                return frames
            body = bytes(self.buf[2:5 + n])
            if checksum(body) == (self.buf[5 + n], self.buf[6 + n]):
                self.good += 1
                frames.append((struct.unpack("<H", body[:2])[0], parse_payload(body[3:])))
                del self.buf[:7 + n]
            else:
                self.bad += 1
                del self.buf[:2]


def self_test():
    p = Parser()
    fr = p.feed(b"\x00\x11" + EXAMPLE + EXAMPLE[:10])
    assert p.good == 1 and p.bad == 0, "decoder self-test: checksum"
    d = fr[0][1]
    assert abs(d[0x10][0] - (-0.122565)) < 1e-9, "decoder self-test: acceleration"
    assert abs(d[0x20][1] - 0.373242) < 1e-9, "decoder self-test: angular rate"
    assert abs(d[0x31][0] - 133.3) < 1e-6, "decoder self-test: raw mag"
    assert abs(d[0x40][2] - (-155.83176)) < 1e-6, "decoder self-test: yaw"


def find_ports():
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))


def usb_info(port):
    """VID, PID, serial and product of the USB adapter behind a tty, from sysfs."""
    dev = os.path.realpath(f"/sys/class/tty/{os.path.basename(port)}/device")
    for _ in range(6):
        if os.path.exists(os.path.join(dev, "idVendor")):
            rd = lambda f: open(os.path.join(dev, f)).read().strip() if os.path.exists(os.path.join(dev, f)) else ""
            return {k: rd(k) for k in ("idVendor", "idProduct", "serial", "product", "manufacturer")}
        dev = os.path.dirname(dev)
    return {}


def read(port, baud, seconds):
    import serial
    p = Parser()
    frames, nbytes = [], 0
    with serial.Serial(port, baud, timeout=0.2) as s:
        s.reset_input_buffer()
        t0 = time.time()
        while time.time() - t0 < seconds:
            chunk = s.read(4096)
            nbytes += len(chunk)
            frames += p.feed(chunk)
    return frames, nbytes, p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="default: the only /dev/ttyUSB* or /dev/ttyACM*")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--scan", action="store_true", help="try 460800, 921600, 230400, 115200, 9600")
    ap.add_argument("--udev", action="store_true", help="print a udev rule naming this adapter /dev/gdn_imu")
    a = ap.parse_args()
    self_test()

    ports = [a.port] if a.port else find_ports()
    if not ports:
        sys.exit("NO PORT: no /dev/ttyUSB* or /dev/ttyACM*. The USB adapter isn't seen by the PC. "
                 "Check the USB cable (a charge-only cable won't work) and see docs/hardware/H30WP_SETUP.md, step 4.")
    if len(ports) > 1:
        sys.exit(f"Several serial ports: {', '.join(ports)}. Unplug the others or pass --port.")
    port = ports[0]
    info = usb_info(port)
    print(f"port {port}: USB {info.get('idVendor', '?')}:{info.get('idProduct', '?')} "
          f"{info.get('manufacturer', '')} {info.get('product', '')} serial {info.get('serial') or 'none'}")
    if a.udev:
        if not info:
            sys.exit("couldn't read the USB details for this port")
        rule = (f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{info["idVendor"]}", ATTRS{{idProduct}}=="{info["idProduct"]}"'
                + (f', ATTRS{{serial}}=="{info["serial"]}"' if info.get("serial") else "")
                + ', SYMLINK+="gdn_imu", MODE="0666"')
        print("\nSave this line as /etc/udev/rules.d/99-gdn-imu.rules, then run:")
        print("  sudo udevadm control --reload-rules && sudo udevadm trigger\n")
        print(rule)
        return
    if not os.access(port, os.R_OK | os.W_OK):
        sys.exit(f"NO PERMISSION on {port}. Run: sudo usermod -aG dialout $USER, then log out and back in.")

    bauds = [a.baud] if not a.scan else [460800, 921600, 230400, 115200, 9600]
    for baud in bauds:
        frames, nbytes, p = read(port, baud, a.seconds if not a.scan else 2.0)
        print(f"baud {baud}: {nbytes} bytes, {p.good} good frames, {p.bad} bad checksums")
        if frames:
            break
    if nbytes == 0:
        sys.exit("NO DATA: the adapter works but nothing arrives from the IMU. Either the IMU has no power, "
                 "or TX and RX are swapped. See docs/hardware/H30WP_SETUP.md, step 6.")
    if not frames:
        sys.exit("BYTES BUT NO FRAMES: data arrives but at another speed or format. Run with --scan.")

    rate = len(frames) / (a.seconds if not a.scan else 2.0)
    seq = [f[0] for f in frames]
    lost = sum(((b - a_) % 60000) - 1 for a_, b in zip(seq, seq[1:]) if ((b - a_) % 60000) != 1)
    have = sorted({k for _, d in frames for k in d})
    print(f"frames: {rate:.0f} per second, {lost} lost (frame-number gaps); contents: "
          + ", ".join(NAMES.get(k, hex(k)) for k in have))
    last = frames[-1][1]

    def mean(did):
        v = [d[did] for _, d in frames if did in d and isinstance(d[did], list)]
        return [sum(x) / len(v) for x in zip(*v)] if v else None
    acc, gyr, mag = mean(0x10), mean(0x20), mean(0x31)
    ok = True
    if acc:
        g = sum(x * x for x in acc) ** 0.5
        good_g = 9.6 < g < 10.0
        ok &= good_g
        print(f"acceleration (m/s^2, FLU): {acc[0]:+.3f} {acc[1]:+.3f} {acc[2]:+.3f}  |a| = {g:.3f}"
              f"  {'OK (about 9.8 at rest)' if good_g else 'CHECK: expected about 9.8 at rest'}")
    else:
        ok = False
        print("acceleration: not in the output (the IMU's output settings need changing; see the guide)")
    if gyr:
        w = sum(x * x for x in gyr) ** 0.5
        still = w < 1.0
        ok &= still
        print(f"angular rate (deg/s, FLU): {gyr[0]:+.4f} {gyr[1]:+.4f} {gyr[2]:+.4f}"
              f"  {'OK (still)' if still else 'CHECK: was it moving?'}")
    else:
        ok = False
        print("angular rate: not in the output")
    if mag:
        print(f"magnetic field (mGauss, raw): {mag[0]:+.1f} {mag[1]:+.1f} {mag[2]:+.1f}  "
              f"|m| = {sum(x * x for x in mag) ** 0.5:.0f} (Earth is roughly 250 to 650)")
    if 0x01 in last:
        print(f"temperature: {last[0x01]:.1f} C")
    if 0x40 in last:
        e = last[0x40]
        print(f"IMU's own attitude: pitch {e[0]:+.2f}, roll {e[1]:+.2f}, yaw {e[2]:+.2f} deg")
    if 0x51 not in have:
        print("note: no sample timestamp in the output; it helps for timing, see the guide")
    print("\nRESULT:", "PASS, the IMU works" if ok and lost == 0 else
          "PASS with notes above" if ok else "NOT OK, see the CHECK lines above")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
