"""Patrol area file and OSM helpers for the UAV patrol mainline."""

import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from uavpatrol_navigation.geo_utils import latlon_to_local


def find_project_root():
    env_root = os.environ.get("ROBOTDOG_ROOT") or os.environ.get("UAVPATROL_ROOT")
    if env_root:
        return Path(env_root)
    p = Path(__file__).resolve()
    for parent in [p] + list(p.parents):
        if (parent / "data").is_dir() and (parent / "src").is_dir():
            return parent
    return Path.cwd()


def resolve_project_path(path_value):
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return path
    return find_project_root() / path


def load_patrol_area(path_value):
    path = resolve_project_path(path_value)
    data = json.loads(path.read_text(encoding="utf-8"))
    boundary = data.get("boundary", [])
    frame = str(data.get("coordinate_frame", "WGS84")).upper()
    if len(boundary) < 3:
        raise ValueError(f"{path} must contain at least 3 boundary points")
    return data, path, frame


def patrol_area_to_local_vertices(area_data, origin_lat, origin_lon):
    frame = str(area_data.get("coordinate_frame", "WGS84")).upper()
    vertices = []
    for point in area_data.get("boundary", []):
        if frame in ("WGS84", "CGCS2000"):
            lat = float(point["latitude"])
            lon = float(point["longitude"])
            x, y, _ = latlon_to_local(lat, lon, origin_lat, origin_lon, 0.0)
        elif frame in ("WORLD", "LOCAL", "ENU"):
            x = float(point["x"])
            y = float(point["y"])
        else:
            raise ValueError(f"unsupported patrol area coordinate_frame: {frame}")
        vertices.append((x, y))
    if len(vertices) >= 2 and vertices[0] == vertices[-1]:
        vertices.pop()
    return vertices


def _signed_area(points):
    total = 0.0
    for (lat1, lon1), (lat2, lon2) in zip(points, points[1:] + points[:1]):
        x1 = lon1 * math.cos(math.radians(lat1))
        x2 = lon2 * math.cos(math.radians(lat2))
        total += x1 * lat2 - x2 * lat1
    return total * 0.5


def _way_points(way, nodes):
    refs = [node_ref.get("ref") for node_ref in way.findall("nd")]
    points = [nodes[ref] for ref in refs if ref in nodes]
    if len(points) >= 2 and points[0] == points[-1]:
        points = points[:-1]
    return points


def extract_patrol_area_from_osm(osm_path_value):
    """Extract a conservative WGS84 patrol polygon from local OSM data.

    Preference is given to closed education/university/school boundaries. If no
    semantic boundary is available, the function falls back to a smaller
    rectangle inside the OSM bounds, so tests stay on the mapped campus area.
    """
    osm_path = resolve_project_path(osm_path_value)
    root = ET.parse(osm_path).getroot()
    nodes = {
        node.get("id"): (float(node.get("lat")), float(node.get("lon")))
        for node in root.findall("node")
    }

    candidates = []
    for way in root.findall("way"):
        tags = {tag.get("k"): tag.get("v") for tag in way.findall("tag")}
        semantic_match = (
            tags.get("landuse") == "education"
            or tags.get("amenity") in ("university", "school", "college")
        )
        if not semantic_match:
            continue
        points = _way_points(way, nodes)
        if len(points) < 3:
            continue
        area = abs(_signed_area(points))
        candidates.append((area, tags, points))

    if candidates:
        _, tags, points = max(candidates, key=lambda item: item[0])
        name = (
            tags.get("name:en")
            or tags.get("name")
            or tags.get("amenity")
            or "campus_patrol_area"
        )
        return {
            "area_name": str(name),
            "coordinate_frame": "WGS84",
            "boundary": [
                {"latitude": lat, "longitude": lon}
                for lat, lon in points
            ],
        }, "semantic_boundary"

    bounds = root.find("bounds")
    if bounds is None:
        lats = [lat for lat, _ in nodes.values()]
        lons = [lon for _, lon in nodes.values()]
        minlat, maxlat = min(lats), max(lats)
        minlon, maxlon = min(lons), max(lons)
    else:
        minlat = float(bounds.get("minlat"))
        maxlat = float(bounds.get("maxlat"))
        minlon = float(bounds.get("minlon"))
        maxlon = float(bounds.get("maxlon"))

    lat_margin = (maxlat - minlat) * 0.25
    lon_margin = (maxlon - minlon) * 0.25
    rectangle = [
        (minlat + lat_margin, minlon + lon_margin),
        (minlat + lat_margin, maxlon - lon_margin),
        (maxlat - lat_margin, maxlon - lon_margin),
        (maxlat - lat_margin, minlon + lon_margin),
    ]
    return {
        "area_name": "campus_patrol_area",
        "coordinate_frame": "WGS84",
        "boundary": [
            {"latitude": lat, "longitude": lon}
            for lat, lon in rectangle
        ],
    }, "osm_bounds_fallback"
