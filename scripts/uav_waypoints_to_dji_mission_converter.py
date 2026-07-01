#!/usr/bin/env python3
"""CLI wrapper for the DJI mission converter package module."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = ROOT / "src" / "uavpatrol_navigation"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from uavpatrol_navigation.uav_waypoints_to_dji_mission_converter import main


if __name__ == "__main__":
    raise SystemExit(main())
