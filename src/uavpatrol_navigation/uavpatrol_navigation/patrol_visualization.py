"""Generate a lightweight HTML visualization for patrol area and UAV route."""

import html
import json
import zipfile
from pathlib import Path

from uavpatrol_navigation.patrol_area_io import (
    load_patrol_area,
    resolve_project_path,
)


def _load_waypoints(path_value):
    path = resolve_project_path(path_value)
    data = json.loads(path.read_text(encoding="utf-8"))
    return data, path


def _load_optional_geojson(path_value):
    path = resolve_project_path(path_value)
    if not path.exists():
        return {"type": "FeatureCollection", "features": []}
    return json.loads(path.read_text(encoding="utf-8"))


def create_visualization(
    map_file="data/map.osm",
    patrol_area_file="data/input/patrol_area.json",
    waypoint_file="data/output/uav_waypoints.json",
    output_file="data/output/patrol_route_visualization.html",
):
    area, area_path, _ = load_patrol_area(patrol_area_file)
    waypoints_data, waypoint_path = _load_waypoints(waypoint_file)
    output_path = resolve_project_path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    boundary = [
        [float(point["latitude"]), float(point["longitude"])]
        for point in area.get("boundary", [])
    ]
    waypoints = [
        [
            float(point["latitude"]),
            float(point["longitude"]),
            int(point.get("index", idx)),
        ]
        for idx, point in enumerate(waypoints_data.get("waypoints", []))
    ]
    if not boundary:
        raise ValueError(f"{area_path} has empty boundary")
    if not waypoints:
        raise ValueError(f"{waypoint_path} has empty waypoints")

    output_dir = output_path.parent
    static_obstacles = _load_optional_geojson(output_dir / "static_obstacles.geojson")
    obstacle_buffers = _load_optional_geojson(output_dir / "static_obstacle_buffers.geojson")
    target_area = _load_optional_geojson(output_dir / "target_area.geojson")
    coverage_target_area = _load_optional_geojson(output_dir / "coverage_target_area.geojson")
    planning_airspace = _load_optional_geojson(output_dir / "planning_airspace.geojson")
    transit_segments = _load_optional_geojson(output_dir / "transit_segments.geojson")

    all_lat = [point[0] for point in boundary] + [point[0] for point in waypoints]
    all_lon = [point[1] for point in boundary] + [point[1] for point in waypoints]
    center = [(min(all_lat) + max(all_lat)) / 2.0, (min(all_lon) + max(all_lon)) / 2.0]

    html_text = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>UAV Patrol Route</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <style>
    html, body, #map {{ height: 100%; margin: 0; }}
    .label {{
      background: #111827;
      color: white;
      border: 0;
      border-radius: 4px;
      padding: 2px 5px;
      font: 12px/1.2 sans-serif;
    }}
  </style>
</head>
<body>
<div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const boundary = {json.dumps(boundary, ensure_ascii=False)};
const waypoints = {json.dumps(waypoints, ensure_ascii=False)};
const staticObstacles = {json.dumps(static_obstacles, ensure_ascii=False)};
const obstacleBuffers = {json.dumps(obstacle_buffers, ensure_ascii=False)};
const targetArea = {json.dumps(target_area, ensure_ascii=False)};
const coverageTargetArea = {json.dumps(coverage_target_area, ensure_ascii=False)};
const planningAirspace = {json.dumps(planning_airspace, ensure_ascii=False)};
const transitSegments = {json.dumps(transit_segments, ensure_ascii=False)};
const map = L.map('map').setView({json.dumps(center)}, 16);
L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
  maxZoom: 20,
  attribution: '&copy; OpenStreetMap contributors'
}}).addTo(map);

const area = L.polygon(boundary, {{
  color: '#2563eb',
  weight: 3,
  fillColor: '#93c5fd',
  fillOpacity: 0.18
}}).addTo(map).bindPopup({json.dumps(html.escape(area.get("area_name", "patrol_area")))});

const planningLayer = L.geoJSON(planningAirspace, {{
  style: {{
    color: '#64748b',
    weight: 2,
    dashArray: '8 6',
    fillColor: '#cbd5e1',
    fillOpacity: 0.14
  }}
}}).addTo(map).bindPopup('Planning airspace');

const targetLayer = L.geoJSON(targetArea, {{
  style: {{
    color: '#2563eb',
    weight: 3,
    fillColor: '#93c5fd',
    fillOpacity: 0.10
  }}
}}).addTo(map).bindPopup('Target area');

const coverageLayer = L.geoJSON(coverageTargetArea, {{
  style: {{
    color: '#16a34a',
    weight: 2,
    fillColor: '#86efac',
    fillOpacity: 0.18
  }}
}}).addTo(map).bindPopup('Coverage target area');

const bufferLayer = L.geoJSON(obstacleBuffers, {{
  style: {{
    color: '#f97316',
    weight: 2,
    fillColor: '#fdba74',
    fillOpacity: 0.28
  }}
}}).addTo(map).bindPopup('Building safety buffer');

const obstacleLayer = L.geoJSON(staticObstacles, {{
  style: {{
    color: '#7f1d1d',
    weight: 2,
    fillColor: '#ef4444',
    fillOpacity: 0.55
  }}
}}).addTo(map).bindPopup('OSM building');

const routeLatLng = waypoints.map(p => [p[0], p[1]]);
L.polyline(routeLatLng, {{
  color: '#dc2626',
  weight: 3,
  opacity: 0.9
}}).addTo(map);

const transitLayer = L.geoJSON(transitSegments, {{
  style: {{
    color: '#7c3aed',
    weight: 5,
    opacity: 0.95
  }}
}}).addTo(map).bindPopup('Transit outside target_area');

waypoints.forEach((p, i) => {{
  const marker = L.circleMarker([p[0], p[1]], {{
    radius: i === 0 || i === waypoints.length - 1 ? 6 : 4,
    color: i === 0 ? '#16a34a' : (i === waypoints.length - 1 ? '#9333ea' : '#dc2626'),
    fillColor: i === 0 ? '#16a34a' : (i === waypoints.length - 1 ? '#9333ea' : '#f97316'),
    fillOpacity: 0.9,
    weight: 2
  }}).addTo(map);
  marker.bindTooltip(String(p[2]), {{
    permanent: true,
    direction: 'top',
    className: 'label',
    offset: [0, -6]
  }});
  if (i === 0) marker.bindPopup('Start waypoint 0');
  if (i === waypoints.length - 1) marker.bindPopup('End waypoint ' + p[2]);
}});

L.marker(routeLatLng[0]).addTo(map).bindPopup('Start');
L.marker(routeLatLng[routeLatLng.length - 1]).addTo(map).bindPopup('End');
map.fitBounds(L.latLngBounds(boundary.concat(routeLatLng)).pad(0.08));

L.control.layers(null, {{
  'Patrol area': area,
  'Target area': targetLayer,
  'Planning airspace': planningLayer,
  'Coverage target area': coverageLayer,
  'Building buffers': bufferLayer,
  'OSM buildings': obstacleLayer,
  'Outside-target transit': transitLayer
}}, {{collapsed: false}}).addTo(map);
</script>
</body>
</html>
"""
    output_path.write_text(html_text, encoding="utf-8")

    # Keep the map_file argument meaningful for callers and logs even though
    # the HTML uses OSM web tiles as the background.
    map_path = resolve_project_path(map_file)
    return output_path, area_path, waypoint_path, map_path


def kmz_contains_wpml(kmz_path):
    path = resolve_project_path(kmz_path)
    with zipfile.ZipFile(path, "r") as archive:
        names = set(archive.namelist())
    return "wpmz/template.kml" in names and "wpmz/waylines.wpml" in names
