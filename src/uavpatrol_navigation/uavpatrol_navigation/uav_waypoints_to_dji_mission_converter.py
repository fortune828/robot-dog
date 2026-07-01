"""Convert exported UAV GPS waypoints into a DJI WPML/KMZ mission file."""

from __future__ import annotations

import argparse
import json
import math
import time
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

from uavpatrol_navigation.planning_defaults import DEFAULT_PLANNING


KML_NS = "http://www.opengis.net/kml/2.2"
WPML_NS = "http://www.dji.com/wpmz/1.0.6"
MIN_WAYPOINTS = 2
MAX_WAYPOINTS = 65535

ET.register_namespace("", KML_NS)
ET.register_namespace("wpml", WPML_NS)


def _kml(tag: str) -> str:
    return f"{{{KML_NS}}}{tag}"


def _wpml(tag: str) -> str:
    return f"{{{WPML_NS}}}{tag}"


def _add(parent, tag: str, text=None):
    elem = ET.SubElement(parent, tag)
    if text is not None:
        elem.text = str(text)
    return elem


def _fmt_float(value: float, digits: int = 8) -> str:
    return f"{float(value):.{digits}f}".rstrip("0").rstrip(".")


def _indent_xml(elem) -> bytes:
    ET.indent(elem, space="  ")
    return ET.tostring(elem, encoding="utf-8", xml_declaration=True)


@dataclass(frozen=True)
class DjiWaypoint:
    index: int
    latitude: float
    longitude: float
    execute_height_m: float
    waypoint_speed_mps: float


@dataclass(frozen=True)
class DjiMission:
    mission_name: str
    mission_type: str
    aircraft_model: str
    coordinate_frame: str
    altitude_mode: str
    finish_action: str
    global_speed_mps: float
    wayline_id: int
    waypoints: list[DjiWaypoint]


def load_uav_waypoints(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"missing waypoint input: {path}")
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def build_draft(
    source: dict,
    *,
    mission_name: str = "uav_area_patrol",
    finish_action: str = "go_home",
    wayline_id: int = 0,
) -> DjiMission:
    raw_waypoints = source.get("waypoints") or []
    if not isinstance(raw_waypoints, list):
        raise ValueError("input 'waypoints' must be a list")

    coordinate_frame = source.get("coordinate_frame", "WGS84")
    altitude_mode = source.get("altitude_mode", "relative_to_takeoff")
    aircraft_model = source.get("aircraft_model", "DJI Matrice 4E")
    mission_type = source.get("protocol", "DJI_WAYPOINT_3_0")

    if coordinate_frame != "WGS84":
        raise ValueError("first WPML/KMZ version only writes WGS84 coordinates")
    if altitude_mode != "relative_to_takeoff":
        raise ValueError(
            "first WPML/KMZ version only supports relative_to_takeoff altitude"
        )

    waypoints: list[DjiWaypoint] = []
    for expected_index, item in enumerate(raw_waypoints):
        index = int(item.get("index", expected_index))
        latitude = float(item["latitude"])
        longitude = float(item["longitude"])
        height = float(
            item.get(
                "altitude_m",
                item.get("execute_height_m", DEFAULT_PLANNING.altitude_m),
            )
        )
        speed = float(
            item.get(
                "speed_mps",
                item.get("waypoint_speed_mps", DEFAULT_PLANNING.speed_mps),
            )
        )
        waypoints.append(
            DjiWaypoint(
                index=index,
                latitude=latitude,
                longitude=longitude,
                execute_height_m=height,
                waypoint_speed_mps=speed,
            )
        )

    validate_waypoints(waypoints)
    global_speed = float(source.get("global_speed_mps", waypoints[0].waypoint_speed_mps))

    return DjiMission(
        mission_name=mission_name,
        mission_type=mission_type,
        aircraft_model=aircraft_model,
        coordinate_frame=coordinate_frame,
        altitude_mode=altitude_mode,
        finish_action=finish_action,
        global_speed_mps=global_speed,
        wayline_id=int(wayline_id),
        waypoints=waypoints,
    )


def validate_waypoints(waypoints: Iterable[DjiWaypoint]) -> None:
    waypoints = list(waypoints)
    count = len(waypoints)
    if count < MIN_WAYPOINTS:
        raise ValueError(f"DJI Waypoint 3.0 requires at least {MIN_WAYPOINTS} waypoints")
    if count > MAX_WAYPOINTS:
        raise ValueError(f"DJI Waypoint 3.0 supports at most {MAX_WAYPOINTS} waypoints")

    for expected_index, waypoint in enumerate(waypoints):
        if waypoint.index != expected_index:
            raise ValueError(
                f"waypoint indexes must be continuous from 0; "
                f"got {waypoint.index} at position {expected_index}"
            )
        if not math.isfinite(waypoint.latitude) or not (-90.0 <= waypoint.latitude <= 90.0):
            raise ValueError(f"invalid latitude at waypoint {waypoint.index}")
        if not math.isfinite(waypoint.longitude) or not (-180.0 <= waypoint.longitude <= 180.0):
            raise ValueError(f"invalid longitude at waypoint {waypoint.index}")
        if not math.isfinite(waypoint.execute_height_m) or waypoint.execute_height_m <= 0.0:
            raise ValueError(f"invalid execute height at waypoint {waypoint.index}")
        if not math.isfinite(waypoint.waypoint_speed_mps) or waypoint.waypoint_speed_mps <= 0.0:
            raise ValueError(f"invalid speed at waypoint {waypoint.index}")


def mission_to_draft_dict(mission: DjiMission) -> dict:
    return {
        "mission_name": mission.mission_name,
        "mission_type": mission.mission_type,
        "aircraft_model": mission.aircraft_model,
        "coordinate_frame": mission.coordinate_frame,
        "altitude_mode": mission.altitude_mode,
        "finish_action": mission.finish_action,
        "global_speed_mps": mission.global_speed_mps,
        "wayline_id": mission.wayline_id,
        "waypoints": [
            {
                "index": wp.index,
                "latitude": wp.latitude,
                "longitude": wp.longitude,
                "execute_height_m": wp.execute_height_m,
                "waypoint_speed_mps": wp.waypoint_speed_mps,
            }
            for wp in mission.waypoints
        ],
    }


def write_draft_json(mission: DjiMission, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(mission_to_draft_dict(mission), f, ensure_ascii=False, indent=2)
        f.write("\n")


def _wpml_finish_action(finish_action: str) -> str:
    mapping = {
        "go_home": "goHome",
        "no_action": "noAction",
        "auto_land": "autoLand",
        "goto_first_waypoint": "gotoFirstWaypoint",
    }
    if finish_action not in mapping:
        raise ValueError(f"unsupported finish_action: {finish_action}")
    return mapping[finish_action]


def _wpml_height_mode(altitude_mode: str) -> str:
    if altitude_mode == "relative_to_takeoff":
        return "relativeToStartPoint"
    if altitude_mode == "absolute_amsl":
        return "WGS84"
    raise ValueError(f"unsupported altitude_mode: {altitude_mode}")


def _add_mission_config(parent, mission: DjiMission, *, include_rth_height: bool) -> None:
    config = _add(parent, _wpml("missionConfig"))
    _add(config, _wpml("flyToWaylineMode"), "safely")
    _add(config, _wpml("finishAction"), _wpml_finish_action(mission.finish_action))
    _add(config, _wpml("exitOnRCLost"), "goContinue")
    _add(config, _wpml("executeRCLostAction"), "hover")
    safe_height = max(20.0, mission.waypoints[0].execute_height_m)
    _add(config, _wpml("takeOffSecurityHeight"), _fmt_float(safe_height, 2))
    _add(config, _wpml("globalTransitionalSpeed"), _fmt_float(mission.global_speed_mps, 2))
    if include_rth_height:
        _add(config, _wpml("globalRTHHeight"), _fmt_float(safe_height, 2))


def _add_coordinate_sys(parent, mission: DjiMission) -> None:
    coord = _add(parent, _wpml("waylineCoordinateSysParam"))
    _add(coord, _wpml("coordinateMode"), mission.coordinate_frame)
    _add(coord, _wpml("heightMode"), _wpml_height_mode(mission.altitude_mode))
    _add(coord, _wpml("positioningType"), "GPS")


def _add_template_placemark(parent, waypoint: DjiWaypoint) -> None:
    placemark = _add(parent, _kml("Placemark"))
    point = _add(placemark, _kml("Point"))
    _add(
        point,
        _kml("coordinates"),
        f"{_fmt_float(waypoint.longitude)},{_fmt_float(waypoint.latitude)}",
    )
    _add(placemark, _wpml("index"), waypoint.index)
    _add(placemark, _wpml("useGlobalHeight"), 0)
    _add(placemark, _wpml("ellipsoidHeight"), _fmt_float(waypoint.execute_height_m, 2))
    _add(placemark, _wpml("height"), _fmt_float(waypoint.execute_height_m, 2))
    _add(placemark, _wpml("useGlobalSpeed"), 0)
    _add(placemark, _wpml("waypointSpeed"), _fmt_float(waypoint.waypoint_speed_mps, 2))
    _add(placemark, _wpml("useGlobalHeadingParam"), 1)
    _add(placemark, _wpml("useGlobalTurnParam"), 1)


def _add_wayline_placemark(parent, waypoint: DjiWaypoint) -> None:
    placemark = _add(parent, _kml("Placemark"))
    point = _add(placemark, _kml("Point"))
    _add(
        point,
        _kml("coordinates"),
        f"{_fmt_float(waypoint.longitude)},{_fmt_float(waypoint.latitude)}",
    )
    _add(placemark, _wpml("index"), waypoint.index)
    _add(placemark, _wpml("executeHeight"), _fmt_float(waypoint.execute_height_m, 2))
    _add(placemark, _wpml("waypointSpeed"), _fmt_float(waypoint.waypoint_speed_mps, 2))

    heading = _add(placemark, _wpml("waypointHeadingParam"))
    _add(heading, _wpml("waypointHeadingMode"), "followWayline")
    _add(heading, _wpml("waypointHeadingAngle"), 0)
    _add(heading, _wpml("waypointHeadingPathMode"), "followBadArc")

    turn = _add(placemark, _wpml("waypointTurnParam"))
    _add(turn, _wpml("waypointTurnMode"), "toPointAndStopWithDiscontinuityCurvature")
    _add(turn, _wpml("waypointTurnDampingDist"), 0)


def build_template_kml(mission: DjiMission) -> bytes:
    root = ET.Element(_kml("kml"))
    document = _add(root, _kml("Document"))
    now_ms = int(time.time() * 1000)
    _add(document, _wpml("author"), "uavpatrol")
    _add(document, _wpml("createTime"), now_ms)
    _add(document, _wpml("updateTime"), now_ms)
    _add_mission_config(document, mission, include_rth_height=False)

    folder = _add(document, _kml("Folder"))
    _add(folder, _wpml("templateType"), "waypoint")
    _add(folder, _wpml("templateId"), 0)
    _add(folder, _wpml("autoFlightSpeed"), _fmt_float(mission.global_speed_mps, 2))
    _add_coordinate_sys(folder, mission)
    _add(folder, _wpml("globalHeight"), _fmt_float(mission.waypoints[0].execute_height_m, 2))

    heading = _add(folder, _wpml("globalWaypointHeadingParam"))
    _add(heading, _wpml("waypointHeadingMode"), "followWayline")
    _add(heading, _wpml("waypointHeadingAngle"), 0)
    _add(heading, _wpml("waypointHeadingPathMode"), "followBadArc")

    _add(folder, _wpml("globalWaypointTurnMode"), "toPointAndStopWithDiscontinuityCurvature")
    _add(folder, _wpml("globalUseStraightLine"), 1)
    _add(folder, _wpml("gimbalPitchMode"), "manual")

    for waypoint in mission.waypoints:
        _add_template_placemark(folder, waypoint)

    return _indent_xml(root)


def build_waylines_wpml(mission: DjiMission) -> bytes:
    root = ET.Element(_kml("kml"))
    document = _add(root, _kml("Document"))
    _add_mission_config(document, mission, include_rth_height=True)

    folder = _add(document, _kml("Folder"))
    _add(folder, _wpml("templateId"), 0)
    _add(folder, _wpml("waylineId"), mission.wayline_id)
    _add(folder, _wpml("autoFlightSpeed"), _fmt_float(mission.global_speed_mps, 2))
    _add(folder, _wpml("executeHeightMode"), _wpml_height_mode(mission.altitude_mode))
    _add_coordinate_sys(folder, mission)

    for waypoint in mission.waypoints:
        _add_wayline_placemark(folder, waypoint)

    return _indent_xml(root)


def write_kmz(mission: DjiMission, kmz_path: Path) -> None:
    kmz_path.parent.mkdir(parents=True, exist_ok=True)
    template = build_template_kml(mission)
    waylines = build_waylines_wpml(mission)
    with zipfile.ZipFile(kmz_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("wpmz/template.kml", template)
        zf.writestr("wpmz/waylines.wpml", waylines)


def validate_kmz_structure(kmz_path: Path) -> list[str]:
    errors: list[str] = []
    if not kmz_path.is_file():
        return [f"missing KMZ: {kmz_path}"]
    with zipfile.ZipFile(kmz_path, "r") as zf:
        names = set(zf.namelist())
        for required in ("wpmz/template.kml", "wpmz/waylines.wpml"):
            if required not in names:
                errors.append(f"missing {required}")
        for required in ("wpmz/template.kml", "wpmz/waylines.wpml"):
            if required in names:
                try:
                    ET.fromstring(zf.read(required))
                except ET.ParseError as exc:
                    errors.append(f"invalid XML in {required}: {exc}")
    return errors


def write_validation_log(log_path: Path, kmz_path: Path, errors: list[str]) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        f"kmz_path: {kmz_path}",
        f"local_structure_validation: {'PASS' if not errors else 'FAIL'}",
    ]
    if errors:
        lines.extend(f"- {err}" for err in errors)
    lines.extend(
        [
            "",
            "msdk_checkValidation: PENDING_APP_SIDE",
            "Call WPMZManager.getInstance().checkValidation(kmzPath) inside the DJI MSDK/Pilot app.",
            "The Python converter cannot execute Android MSDK validation on the ROS2 server.",
        ]
    )
    log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def convert(
    input_path: Path,
    output_dir: Path,
    *,
    mission_name: str = "uav_area_patrol",
    finish_action: str = "go_home",
    wayline_id: int = 0,
) -> tuple[Path, Path, Path]:
    source = load_uav_waypoints(input_path)
    mission = build_draft(
        source,
        mission_name=mission_name,
        finish_action=finish_action,
        wayline_id=wayline_id,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    draft_path = output_dir / "dji_mission_draft.json"
    kmz_path = output_dir / "dji_mission.kmz"
    log_path = output_dir / "dji_mission_validation.log"

    write_draft_json(mission, draft_path)
    write_kmz(mission, kmz_path)
    errors = validate_kmz_structure(kmz_path)
    write_validation_log(log_path, kmz_path, errors)
    if errors:
        raise ValueError("; ".join(errors))
    return draft_path, kmz_path, log_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Convert data/output/uav_waypoints.json to DJI WPML/KMZ."
    )
    parser.add_argument(
        "--input",
        default="data/output/uav_waypoints.json",
        help="Input UAV waypoint JSON from uav_waypoint_exporter_node.",
    )
    parser.add_argument(
        "--output-dir",
        default="data/output",
        help="Output directory for dji_mission_draft.json and dji_mission.kmz.",
    )
    parser.add_argument("--mission-name", default="uav_area_patrol")
    parser.add_argument(
        "--finish-action",
        default="go_home",
        choices=["go_home", "no_action", "auto_land", "goto_first_waypoint"],
    )
    parser.add_argument("--wayline-id", type=int, default=0)
    args = parser.parse_args(argv)

    draft_path, kmz_path, log_path = convert(
        Path(args.input),
        Path(args.output_dir),
        mission_name=args.mission_name,
        finish_action=args.finish_action,
        wayline_id=args.wayline_id,
    )
    print(f"draft: {draft_path}")
    print(f"kmz: {kmz_path}")
    print(f"validation_log: {log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
