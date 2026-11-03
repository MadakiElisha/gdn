#!/usr/bin/env python3
"""Migrate a rosbag2 recorded with px4_msgs 1.15 so it decodes with px4_msgs 1.16.

Between 1.15 and 1.16, VehicleLocalPosition changed layout (fields inserted and
moved), so its CDR bytes from an old bag no longer decode. Every other topic GDN
uses (SensorCombined, VehicleAttitude, ...) kept the same fields and is copied
byte for byte.

The old layout is read from tools/px4_msgs_1_15/<Type>.msg with a small CDR
decoder, then each message is rebuilt with the installed (1.16) Python class and
re-serialised. Fields that are new in 1.16 keep their defaults.

    source env.sh   # px4_msgs 1.16 built
    python3 tools/migrate_bag_px4_msgs.py data/flight001 data/flight001_v116

Writes <out>/.px4_msgs_version so tools can tell a migrated bag apart.
"""
import argparse
import shutil
import struct
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

HERE = Path(__file__).resolve().parent
OLD_DEFS = HERE / "px4_msgs_1_15"
PRIM = {  # type: (struct code, size)
    "bool": ("?", 1), "byte": ("B", 1), "char": ("b", 1),
    "uint8": ("B", 1), "int8": ("b", 1), "uint16": ("H", 2), "int16": ("h", 2),
    "uint32": ("I", 4), "int32": ("i", 4), "uint64": ("Q", 8), "int64": ("q", 8),
    "float32": ("f", 4), "float64": ("d", 8),
}


def parse_msg_def(path):
    """[(name, type, count)] for the data fields of a flat .msg (no nested types)."""
    fields = []
    for raw in Path(path).read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" in line:          # blank, comment or constant
            continue
        typ, name = line.split()[:2]
        count = 1
        if "[" in typ:
            typ, n = typ[:-1].split("[")
            if not n:
                sys.exit(f"{path}: unbounded array {name} not supported")
            count = int(n)
        if typ not in PRIM:
            sys.exit(f"{path}: nested type {typ} not supported")
        fields.append((name, typ, count))
    return fields


def decode_cdr(data, fields):
    """Decode XCDR1 little-endian bytes (4-byte encapsulation header) into a dict."""
    if data[1] != 0x01:
        sys.exit("only little-endian CDR is supported")
    body, off, out = data[4:], 0, {}
    for name, typ, count in fields:
        code, size = PRIM[typ]
        off = (off + size - 1) // size * size       # CDR alignment
        vals = struct.unpack_from("<" + code * count, body, off)
        off += size * count
        out[name] = vals[0] if count == 1 else list(vals)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("in_bag")
    ap.add_argument("out_bag")
    ap.add_argument("--force", action="store_true", help="overwrite out_bag")
    a = ap.parse_args()

    out = Path(a.out_bag)
    if out.exists():
        if not a.force:
            sys.exit(f"{out} exists (use --force)")
        shutil.rmtree(out)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.in_bag, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = reader.get_all_topics_and_types()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))

    convert = {}   # topic -> (old fields, new class)
    for i, t in enumerate(topics):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=t.name, type=t.type, serialization_format=t.serialization_format))
        short = t.type.split("/")[-1]
        old_def = OLD_DEFS / f"{short}.msg"
        if old_def.exists():
            convert[t.name] = (parse_msg_def(old_def), get_message(t.type))

    counts = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in convert:
            fields, cls = convert[topic]
            vals = decode_cdr(data, fields)
            msg = cls()
            for k, v in vals.items():
                if hasattr(msg, k):
                    setattr(msg, k, v)
            data = serialize_message(msg)
        writer.write(topic, data, stamp)
        counts[topic] = counts.get(topic, 0) + 1
    del writer
    (out / ".px4_msgs_version").write_text("1.16\n")
    for t in sorted(counts):
        tag = "converted" if t in convert else "copied"
        print(f"{tag:9s} {counts[t]:7d}  {t}")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
