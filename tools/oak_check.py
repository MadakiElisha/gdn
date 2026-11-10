#!/usr/bin/env python3
"""Check a Luxonis OAK-D Lite on this PC and say plainly whether it's fit for VIO.

    python3 tools/oak_check.py              # device, USB link, cameras, IMU, calibration
    python3 tools/oak_check.py --stream     # plus 5 s of both mono cameras and the IMU

What matters for GDN's stereo VIO (R3.5) and why it's checked:
- the USB link runs at USB 3 (SUPER). On USB 2 the stereo pair can't run at full rate;
- both global-shutter mono cameras (CAM_B left, CAM_C right) are present;
- an IMU is fitted. Early OAK-D Lite batches shipped without one. VIO then has to use
  the H30WP with an estimated time offset instead;
- the factory calibration is on the device: intrinsics, stereo baseline (about 7.5 cm)
  and, if fitted, camera-to-IMU extrinsics.

Needs the DepthAI Python library (python3 -m pip install depthai, v3) and the Luxonis
udev rule; see docs/hardware/OAK_D_LITE_SETUP.md.
"""
import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stream", action="store_true", help="also run both mono cameras and the IMU for 5 s")
    ap.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args()
    try:
        import depthai as dai
    except ImportError:
        sys.exit("NO DEPTHAI: install it with  python3 -m pip install --user depthai  (see the setup guide)")
    print(f"depthai {dai.__version__}")

    devs = dai.Device.getAllAvailableDevices()
    if not devs:
        sys.exit("NO DEVICE: DepthAI can't see a camera. Check the USB cable and the udev rule (setup guide, "
                 "'Preparing the PC'), then unplug and replug the camera.")
    if len(devs) > 1:
        print(f"note: {len(devs)} devices found, checking the first")
    print(f"found: {devs[0].name} ({devs[0].getDeviceId()}), state {devs[0].state}")

    notes, problems = [], []
    with dai.Device() as dev:
        name = dev.getProductName() or dev.getDeviceName()
        speed = dev.getUsbSpeed()
        print(f"product: {name}, id {dev.getDeviceId()}")
        print(f"USB link: {speed.name}")
        if speed not in (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS):
            problems.append("USB link isn't USB 3. Use a USB 3 port (blue, or marked SS) and a USB 3 C cable; "
                            "try the cable the other way round in the camera.")

        feats = dev.getConnectedCameraFeatures()
        for f in feats:
            print(f"camera {f.socket.name}: {f.sensorName} {f.width}x{f.height}")
        sockets = {f.socket for f in feats}
        for s, what in ((dai.CameraBoardSocket.CAM_B, "left mono"), (dai.CameraBoardSocket.CAM_C, "right mono")):
            if s not in sockets:
                problems.append(f"{what} camera ({s.name}) not found")

        imu = dev.getConnectedIMU()
        print(f"IMU: {imu or 'none'}")
        if not imu or imu.upper() == "NONE":
            notes.append("no IMU on this unit (early OAK-D Lite batch). VIO will use the H30WP instead; "
                         "that needs a camera-to-H30WP calibration with a time offset (R6.4).")
            imu = None

        try:
            cal = dev.readCalibration()
            L, R = dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
            t = cal.getCameraTranslationVector(L, R, False)        # measured, not the design value
            base = (t[0] ** 2 + t[1] ** 2 + t[2] ** 2) ** 0.5
            print(f"calibration: on device, stereo baseline {base:.2f} cm")
            if not 6.5 < base < 8.5:
                problems.append(f"stereo baseline {base:.2f} cm, expected about 7.5 cm for an OAK-D Lite")
            if imu:
                try:
                    cal.getCameraToImuExtrinsics(L)
                    print("calibration: camera-to-IMU extrinsics present")
                except Exception:
                    notes.append("no camera-to-IMU extrinsics on the device; we'll calibrate them with Kalibr (R6.4)")
        except Exception as e:
            problems.append(f"couldn't read the factory calibration ({e})")

        if a.stream and not problems:
            stream_test(dai, dev, imu, a.seconds, problems)

    for n in notes:
        print("note:", n)
    for p in problems:
        print("PROBLEM:", p)
    print("\nRESULT:", "PASS" if not problems else "NOT OK, see PROBLEM lines above")
    sys.exit(0 if not problems else 1)


def stream_test(dai, dev, imu, seconds, problems):
    """Both mono cameras at 640x400, 30 fps, and the IMU at about 200 Hz, for a few seconds."""
    with dai.Pipeline(dev) as p:
        qs = {}
        for s in (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C):
            cam = p.create(dai.node.Camera).build(s)
            qs[s.name] = cam.requestOutput((640, 400), fps=30).createOutputQueue(maxSize=8, blocking=False)
        qi = None
        if imu:
            n = p.create(dai.node.IMU)
            n.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
            n.setBatchReportThreshold(1)
            n.setMaxBatchReports(10)
            qi = n.out.createOutputQueue(maxSize=50, blocking=False)
        p.start()
        counts = {k: 0 for k in qs}
        n_imu, acc = 0, []
        t0 = time.time()
        while time.time() - t0 < seconds:
            for k, q in qs.items():
                while q.tryGet() is not None:
                    counts[k] += 1
            if qi is not None:
                m = qi.tryGet()
                while m is not None:
                    for pk in m.packets:
                        n_imu += 1
                        v = pk.acceleroMeter
                        acc.append((v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5)
                    m = qi.tryGet()
            time.sleep(0.002)
        p.stop()
    for k, c in counts.items():
        fps = c / seconds
        print(f"stream {k}: {fps:.1f} fps")
        if fps < 25:
            problems.append(f"{k} ran at {fps:.1f} fps, expected about 30")
    if qi is not None:
        rate = n_imu / seconds
        g = sum(acc) / len(acc) if acc else 0.0
        print(f"stream IMU: {rate:.0f} Hz, |a| = {g:.2f} m/s^2")
        if rate < 150:
            problems.append(f"IMU ran at {rate:.0f} Hz, expected about 200")
        if not 9.3 < g < 10.3:
            problems.append(f"IMU |a| = {g:.2f}, expected about 9.8 with the camera still")


if __name__ == "__main__":
    main()
