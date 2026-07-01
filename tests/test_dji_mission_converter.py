import json
import zipfile
from xml.etree import ElementTree as ET

from uavpatrol_navigation.uav_waypoints_to_dji_mission_converter import convert


def test_convert_uav_waypoints_to_dji_mission(tmp_path):
    input_path = tmp_path / "uav_waypoints.json"
    output_dir = tmp_path / "out"
    input_path.write_text(
        json.dumps(
            {
                "mission_name": "uav_area_mission",
                "protocol": "DJI_WAYPOINT_3_0",
                "aircraft_model": "DJI Matrice 4E",
                "coordinate_frame": "WGS84",
                "altitude_mode": "relative_to_takeoff",
                "waypoints": [
                    {
                        "index": 0,
                        "latitude": 30.0,
                        "longitude": 103.0,
                        "altitude_m": 30.0,
                        "speed_mps": 5.0,
                        "coordinate_frame": "WGS84",
                    },
                    {
                        "index": 1,
                        "latitude": 30.0001,
                        "longitude": 103.0001,
                        "altitude_m": 30.0,
                        "speed_mps": 5.0,
                        "coordinate_frame": "WGS84",
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    draft_path, kmz_path, log_path = convert(input_path, output_dir)

    draft = json.loads(draft_path.read_text(encoding="utf-8"))
    assert draft["mission_type"] == "DJI_WAYPOINT_3_0"
    assert draft["aircraft_model"] == "DJI Matrice 4E"
    assert draft["coordinate_frame"] == "WGS84"
    assert draft["altitude_mode"] == "relative_to_takeoff"
    assert draft["finish_action"] == "go_home"
    assert draft["wayline_id"] == 0
    assert len(draft["waypoints"]) == 2
    assert draft["waypoints"][0]["execute_height_m"] == 30.0

    with zipfile.ZipFile(kmz_path, "r") as zf:
        assert "wpmz/template.kml" in zf.namelist()
        assert "wpmz/waylines.wpml" in zf.namelist()
        ET.fromstring(zf.read("wpmz/template.kml"))
        ET.fromstring(zf.read("wpmz/waylines.wpml"))

    assert "local_structure_validation: PASS" in log_path.read_text(encoding="utf-8")
