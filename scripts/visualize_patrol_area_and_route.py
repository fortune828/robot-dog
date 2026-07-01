#!/usr/bin/env python3
"""Generate HTML visualization for the UAV patrol area and exported route."""

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = ROOT / "src" / "uavpatrol_navigation"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from uavpatrol_navigation.patrol_visualization import create_visualization


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", default="data/map.osm", dest="map_file")
    parser.add_argument("--area", default="data/input/patrol_area.json")
    parser.add_argument("--waypoints", default="data/output/uav_waypoints.json")
    parser.add_argument(
        "--output",
        default="data/output/patrol_route_visualization.html",
    )
    args = parser.parse_args()

    output_path, area_path, waypoint_path, map_path = create_visualization(
        map_file=args.map_file,
        patrol_area_file=args.area,
        waypoint_file=args.waypoints,
        output_file=args.output,
    )
    print(f"map: {map_path}")
    print(f"area: {area_path}")
    print(f"waypoints: {waypoint_path}")
    print(f"output: {output_path}")


if __name__ == "__main__":
    main()
