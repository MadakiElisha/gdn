#!/usr/bin/env python3
"""Record the OAK-D Lite's IMU with its cameras running, for the R1.6 Allan fit.

    ~/.venvs/oak/bin/python tools/oak_imu_log.py --out log/oakd_lite_static3 --hours 2.25
    python3 tools/allan_fit.py --csv log/oakd_lite_static3/imu.csv --skip 1800 --name oakd_lite ...

Why not the ROS driver: the apt driver (depthai-ros v3, DepthAI 3.10) crashes this unit
within seconds ("Start Source: Invalid config steps"), while DepthAI 3.12 runs it cleanly.
This uses DepthAI 3.12 from ~/.venvs/oak (docs/hardware/OAK_D_LITE_SETUP.md).

The three cameras stream as they would in use (colour 1280x720, both mono 640x400, all at
30 fps), so the IMU warms up as it does in flight; their frames are counted and dropped.

imu.csv: one row per IMU report, with the camera's own clock (no USB jitter):
  t       gyro sample time (us, device clock)     gx, gy, gz  rad/s
  t_acc   accel sample time (us, device clock)    ax, ay, az  m/s^2
  seq     report sequence number (a jump means a lost report)
Every minute it prints the IMU rate, frame rates and lost reports. If the camera crashes
it stops: a restart would mean a fresh warm-up in the middle of the log.

--ros publishes a 10 Hz preview while it records (only one program can open the camera):
/oak/rgb/image_raw (640x360), /oak/left/image_raw and /oak/right/image_raw (640x400).
Run it with the ROS Python for that:
    source /opt/ros/jazzy/setup.bash
    python3 tools/oak_imu_log.py --out log/oakd_lite_static4 --hours 2.25 --ros
    ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw      # in another terminal
"""
import argparse
import csv
import glob
import os
import sys
import time

try:
    import depthai  # noqa: F401
except ImportError:  # ROS Python: use the DepthAI 3.12 installed in the setup guide's venv
    sys.path += glob.glob(os.path.expanduser("~/.venvs/oak/lib/python3*/site-packages"))


class Preview:
    """Publish every 3rd camera frame to ROS, and only while someone is watching."""

    def __init__(self, names):
        import rclpy
        from sensor_msgs.msg import Image
        rclpy.init()
        self.Image = Image
        self.node = rclpy.create_node("oak_imu_log_preview")
        topics = {"CAM_A": "/oak/rgb/image_raw", "CAM_B": "/oak/left/image_raw", "CAM_C": "/oak/right/image_raw"}
        self.pubs = {k: self.node.create_publisher(Image, topics[k], 1) for k in names}
        self.count = {k: 0 for k in names}

    def frame(self, name, f):
        self.count[name] += 1
        pub = self.pubs[name]
        if self.count[name] % 3 or pub.get_subscription_count() == 0:
            return
        img = f.getFrame()                       # numpy: HxW (mono) or HxWx3 (BGR)
        if img.ndim == 3:
            img = img[::2, ::2].copy()           # colour preview at half size, made contiguous
        m = self.Image()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = f"oak_{name.lower()}"
        m.height, m.width = img.shape[:2]
        m.encoding = "bgr8" if img.ndim == 3 else "mono8"
        m.step = img.strides[0]
        m.data = img.tobytes()
        pub.publish(m)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output directory (must not exist yet)")
    ap.add_argument("--hours", type=float, default=2.25)
    ap.add_argument("--imu-hz", type=int, default=400)
    ap.add_argument("--no-color", action="store_true", help="don't run the colour camera")
    ap.add_argument("--ros", action="store_true", help="publish a 10 Hz camera preview to ROS")
    a = ap.parse_args()
    import depthai as dai

    os.makedirs(a.out, exist_ok=False)
    path = os.path.join(a.out, "imu.csv")
    print(f"depthai {dai.__version__}, writing {path} for {a.hours} h. Ctrl-C stops early.")

    with dai.Device() as dev, dai.Pipeline(dev) as p:
        print(f"{dev.getProductName()} {dev.getDeviceId()}, USB {dev.getUsbSpeed().name}, IMU {dev.getConnectedIMU()}")
        cams = [(dai.CameraBoardSocket.CAM_B, (640, 400)), (dai.CameraBoardSocket.CAM_C, (640, 400))]
        if not a.no_color:
            cams.insert(0, (dai.CameraBoardSocket.CAM_A, (1280, 720)))
        qs = {}
        for s, size in cams:
            cam = p.create(dai.node.Camera).build(s)
            kind = dai.ImgFrame.Type.BGR888i if s == dai.CameraBoardSocket.CAM_A else dai.ImgFrame.Type.GRAY8
            qs[s.name] = cam.requestOutput(size, type=kind, fps=30).createOutputQueue(maxSize=4, blocking=False)
        preview = Preview(list(qs)) if a.ros else None
        imu = p.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], a.imu_hz)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(20)
        qi = imu.out.createOutputQueue(maxSize=200, blocking=True)
        p.start()

        frames = {k: 0 for k in qs}
        n = lost = 0
        last_seq = None
        t0 = time.time()
        t_end = t0 + a.hours * 3600
        t_rep, n_rep, f_rep = t0, 0, dict(frames)
        us = lambda ts: f"{ts.getTimestampDevice().total_seconds() * 1e6:.0f}"
        with open(path, "w", newline="") as fo:
            w = csv.writer(fo)
            w.writerow(["t", "gx", "gy", "gz", "t_acc", "ax", "ay", "az", "seq"])
            try:
                while time.time() < t_end:
                    if not p.isRunning():
                        sys.exit("STOPPED: the camera pipeline stopped (device crash?). Data so far is kept.")
                    m = qi.tryGet()
                    while m is not None:
                        for pk in m.packets:
                            g, acc = pk.gyroscope, pk.acceleroMeter
                            if last_seq is not None and g.sequence < last_seq:
                                # The camera restarted (DepthAI reconnects by itself): its clock
                                # and counter begin again, and so does its warm-up (TEST-025).
                                fo.flush()
                                sys.exit(f"STOPPED: the camera restarted {(time.time() - t0) / 60:.1f} min in "
                                         f"(report counter {last_seq} -> {g.sequence}). Data up to the restart "
                                         f"is kept in {path}.")
                            if last_seq is not None and g.sequence > last_seq + 1:
                                lost += g.sequence - last_seq - 1
                            last_seq = g.sequence
                            w.writerow([us(g), f"{g.x:.7f}", f"{g.y:.7f}", f"{g.z:.7f}",
                                        us(acc), f"{acc.x:.6f}", f"{acc.y:.6f}", f"{acc.z:.6f}", g.sequence])
                            n += 1
                        m = qi.tryGet()
                    for k, q in qs.items():
                        f = q.tryGet()
                        while f is not None:
                            frames[k] += 1
                            if preview:
                                preview.frame(k, f)
                            f = q.tryGet()
                    now = time.time()
                    if now - t_rep >= 60:
                        dt = now - t_rep
                        fps = ", ".join(f"{k} {(frames[k] - f_rep[k]) / dt:.1f}" for k in frames)
                        print(f"{time.strftime('%H:%M:%S')}  {(now - t0) / 60:6.1f} min  IMU {(n - n_rep) / dt:.1f} Hz"
                              f"  lost {lost}  fps: {fps}", flush=True)
                        fo.flush()
                        t_rep, n_rep, f_rep = now, n, dict(frames)
                    time.sleep(0.002)
            except KeyboardInterrupt:
                print("stopped by Ctrl-C")
        p.stop()
    print(f"done: {n} IMU reports, {lost} lost, {(time.time() - t0) / 3600:.2f} h, in {path}")


if __name__ == "__main__":
    main()
