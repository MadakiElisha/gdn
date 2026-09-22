#!/usr/bin/env python3
import os
import sys
import argparse
import elevation

def main():
    parser = argparse.ArgumentParser(description="Fetch and stitch real-world SRTM terrain data.")
    parser.add_argument("--lat", type=float, required=True, help="Center latitude")
    parser.add_argument("--lon", type=float, required=True, help="Center longitude")
    parser.add_argument("--radius", type=float, default=0.05, help="Radius in degrees (~5.5km per 0.05 deg)")
    parser.add_argument("--out", type=str, default="terrain.tif", help="Output GeoTIFF file")
    args = parser.parse_args()

    # FIX: Convert to absolute path so gdal_translate finds it from the cache dir
    out_path = os.path.abspath(args.out)
    out_dir = os.path.dirname(out_path)
    os.makedirs(out_dir, exist_ok=True)

    west = args.lon - args.radius
    south = args.lat - args.radius
    east = args.lon + args.radius
    north = args.lat + args.radius

    print(f"Fetching SRTM tiles for bounding box: W={west}, S={south}, E={east}, N={north}")
    print("Processing cached tiles...")
    
    try:
        elevation.clip(bounds=(west, south, east, north), output=out_path)
        print(f"Successfully saved terrain to {out_path}")
    except Exception as e:
        print(f"Error fetching terrain: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
