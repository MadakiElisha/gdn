#!/usr/bin/env python3
"""Network check and driver config for the Livox Mid-360S.

    python3 tools/livox_setup.py --sn 47MDL1C0010081      # the serial on the lidar's label
    python3 tools/livox_setup.py --ip 192.168.1.181       # if you already know its IP

The Mid-360S leaves the factory at 192.168.1.1XX, where XX is the last two digits of
its serial number. The PC needs a fixed address on the same subnet, on the Ethernet
port the lidar is plugged into (we use 192.168.1.50). This script:

1. works out the lidar's IP from the serial number;
2. checks that one of the PC's interfaces has a 192.168.1.x/24 address;
3. pings the lidar;
4. writes config/sensors/mid360s.json for livox_ros_driver2 (the format of the
   driver's own MID360s_config.json), with the lidar and host IPs filled in.

It prints a plain verdict at each step. tools/livox_run.sh then starts the driver with
that config. See docs/hardware/MID360S_SETUP.md.
"""
import argparse
import ipaddress
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "config/sensors/mid360s.json"


def host_addresses():
    """(interface, IPv4Interface) for every IPv4 address on this machine."""
    try:
        out = subprocess.run(["ip", "-j", "-4", "addr"], capture_output=True, text=True).stdout
    except FileNotFoundError:
        sys.exit("the 'ip' command is missing: sudo apt install iproute2")
    res = []
    for itf in json.loads(out or "[]"):
        for ad in itf.get("addr_info", []):
            res.append((itf["ifname"], ipaddress.ip_interface(f"{ad['local']}/{ad['prefixlen']}")))
    return res


def config(lidar_ip, host_ip):
    host = {"host_ip": host_ip, "cmd_data_port": 56101, "push_msg_port": 56201,
            "point_data_port": 56301, "imu_data_port": 56401, "log_data_port": 56501}
    return {
        "lidar_summary_info": {"lidar_type": 8},
        "Mid360s": {
            "lidar_net_info": {"cmd_data_port": 56100, "push_msg_port": 56200, "point_data_port": 56300,
                               "imu_data_port": 56400, "log_data_port": 56500},
            "host_net_info": [host],
        },
        "lidar_configs": [{
            "ip": lidar_ip, "pcl_data_type": 1, "pattern_mode": 0,
            # Mount pose of the lidar on the vehicle; zero until it's mounted (R6.4).
            "extrinsic_parameter": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0, "y": 0, "z": 0},
        }],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sn", help="serial number from the lidar's label")
    g.add_argument("--ip", help="the lidar's IP, if already known")
    ap.add_argument("--host-ip", help="the PC's address to use (default: the one found on 192.168.1.0/24)")
    a = ap.parse_args()

    if a.sn:
        digits = re.sub(r"\D", "", a.sn)
        if len(digits) < 2:
            sys.exit("the serial number needs to end in two digits; check the label")
        lidar_ip = f"192.168.1.1{digits[-2:]}"
        print(f"lidar IP from serial {a.sn}: {lidar_ip}")
    else:
        lidar_ip = a.ip
    lidar = ipaddress.ip_address(lidar_ip)

    on_subnet = [(n, i) for n, i in host_addresses() if lidar in i.network and i.ip != lidar]
    if a.host_ip:
        host_ip = a.host_ip
    elif on_subnet:
        n, i = on_subnet[0]
        host_ip = str(i.ip)
        print(f"PC address on the lidar's subnet: {host_ip} on {n}")
    else:
        sys.exit("NO HOST ADDRESS: no network interface has an address on 192.168.1.0/24. Give the Ethernet port "
                 "the lidar is on a fixed address, 192.168.1.50/24 (setup guide, 'The PC's network').")
    if len(on_subnet) > 1:
        print("note: more than one interface is on 192.168.1.0/24: "
              + ", ".join(f"{n} {i}" for n, i in on_subnet)
              + ". A home router on the same subnet will confuse things; see the guide.")

    try:
        ping = subprocess.run(["ping", "-c", "3", "-W", "1", lidar_ip], capture_output=True, text=True)
    except FileNotFoundError:
        sys.exit("the 'ping' command is missing: sudo apt install iputils-ping")
    if ping.returncode != 0:
        print(f"NO REPLY from {lidar_ip}. Check the lidar has power (it hums slightly and gets warm), the "
              "Ethernet cable, and that the serial number's last two digits are right.")
        print("Writing the config anyway, so it's ready once the lidar answers.")
        ok = False
    else:
        rtt = re.search(r"= [\d.]+/([\d.]+)/", ping.stdout)
        print(f"lidar answers at {lidar_ip}" + (f" ({rtt.group(1)} ms)" if rtt else ""))
        ok = True

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(config(lidar_ip, host_ip), indent=2) + "\n")
    print(f"wrote {OUT.relative_to(ROOT)} (lidar {lidar_ip}, host {host_ip})")
    print("\nRESULT:", "PASS, start the driver with tools/livox_run.sh" if ok else "NOT OK, the lidar didn't answer")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
