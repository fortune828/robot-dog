# uavpatrol

ROS2 UAV patrol workspace.

当前主线：

```text
data/input/patrol_area.json 巡逻区域
-> 空域覆盖规划
-> 生成覆盖航点 /global_plan
-> UAV waypoint JSON/CSV
-> DJI Waypoint 3.0 draft/KMZ
```

本工程当前不直接控制无人机飞控；航线上传、执行、暂停、恢复、停止交给 DJI Pilot 2 / MSDK App 完成。

## Planning Logic

UAV 不是地面机器人，默认不沿 OSM road graph 飞行。当前默认规划入口是：

```text
data/input/patrol_area.json
-> semantic constrained polygon_coverage_planner
-> /global_plan
-> uav_waypoint_exporter_node
-> data/output/uav_waypoints.csv
-> data/output/uav_waypoints.json
-> data/output/dji_mission_draft.json
-> data/output/dji_mission.kmz
-> data/output/patrol_route_visualization.html
```

OSM 在当前主线中的定位：

- 提供地图背景；
- 提供默认巡逻区域来源；
- 提供建筑物、水域、绿地等语义先验；
- 当前已使用 `building=*` 生成静态障碍 buffer；
- 后续可用于生成风险区域；
- 不作为默认导航器；
- 不默认通过 `/plan_osm_path` 生成飞行路径。

## Mainline Packages

```text
src/uavpatrol_navigation
src/uavpatrol_bringup
```

旧 `sanitation_*`、`depth_anything_v3` 等 package 保留为 legacy ground-robot 内容，当前 UAV 主线不从这些包启动。

## Build

```bash
cd /home/ubuntu/bl/workspace/robot-dog
source setup_env.sh
colcon build --symlink-install
source install/setup.bash
```

## Launch

地图手动画小范围巡逻区域：

```bash
python scripts/patrol_area_selector_server.py
```

默认使用：

```text
map: data/map.osm
output: data/input/patrol_area.json
coverage_spacing: 10.0m
building_buffer: 5.0m
min_flyable_area: 5.0m2
route_edge_margin: 3.0m
max_off_area_distance: 0.0m
sensor_coverage_radius: 0.0m
min_coverage_contribution: 1.0m
max_connection_length: 150.0m
```

这些默认值统一定义在 `src/uavpatrol_navigation/uavpatrol_navigation/planning_defaults.py`。

打开 `http://localhost:8000` 后在真实地图上画 polygon，点击 `Save & Generate` 会直接生成语义约束航线、DJI mission/KMZ 和地图叠加结果。

点击 `Save & Generate` 后会自动完成：

```text
保存 data/input/patrol_area.json
-> 读取 data/map.osm 建筑物
-> 只在用户画的 polygon 内扣除 building safety buffer
-> 生成语义约束覆盖航线
-> 生成 DJI mission/KMZ
-> 生成 data/output/patrol_route_visualization.html
```

也就是说，小范围地图选区测试不需要再手动运行 ROS launch。

桌面 demo：

```bash
ros2 launch uavpatrol_bringup uav_waypoint_export.launch.py use_mock_gps:=true
```

默认会读取 `data/input/patrol_area.json` 和 `data/map.osm`，扣除 OSM 建筑物安全 buffer 后，由 `polygon_coverage_planner` 生成 `/global_plan`。`/clicked_point` 仍保留为 RViz 交互输入，但默认不依赖它。

真实 RTK：

```bash
ros2 launch uavpatrol_bringup uav_waypoint_export.launch.py \
  gps_topic:=/fix \
  altitude_m:=30.0 \
  altitude_mode:=relative_to_takeoff \
  speed_mps:=5.0 \
  coordinate_frame:=WGS84
```

真实 RTK 模式下，默认仍读取 `data/input/patrol_area.json`。如需 RViz 交互选区，可加 `area_source:=clicked_point`，闭合方式：最后一个点靠近第一个点，距离小于 `closing_threshold` 后触发覆盖规划。

手动 RViz / demo polygon：

```bash
ros2 launch uavpatrol_bringup uav_waypoint_export.launch.py \
  use_mock_gps:=true \
  area_source:=demo
```

## Outputs

```text
data/output/uav_waypoints.csv
data/output/uav_waypoints.json
data/output/static_obstacles.geojson
data/output/static_obstacle_buffers.geojson
data/output/target_area.geojson
data/output/coverage_target_area.geojson
data/output/planning_airspace.geojson
data/output/flyable_area.geojson
data/output/dji_mission_draft.json
data/output/dji_mission.kmz
data/output/dji_mission_validation.log
data/output/patrol_route_visualization.html
```

If needed, run the converter manually:

```bash
python scripts/uav_waypoints_to_dji_mission_converter.py
```

Regenerate the HTML route view manually:

```bash
python scripts/visualize_patrol_area_and_route.py
```

Run semantic coverage planning without ROS:

```bash
python scripts/run_semantic_coverage_planning.py
```

网页选区工具里也有 `Generate Route` 按钮；它会基于当前 `data/input/patrol_area.json` 重新执行语义覆盖规划，并直接在当前地图上叠加建筑、buffer、可飞区域、航线和航点编号，同时生成 `data/output/patrol_route_visualization.html`。

## Main Nodes

- `osm_map_manager`: 加载 OSM，发布地图/建筑物可视化；默认 `enable_routing: false`。
- `polygon_coverage_planner`: 默认规划入口，读取 `data/input/patrol_area.json` 和 OSM 建筑物语义，输出避开建筑 buffer 的覆盖航线 `/global_plan`。
- `uav_waypoint_exporter_node`: 将 `/global_plan` 转为 WGS84 GPS 航点，并自动生成 DJI mission draft/KMZ。
- `mock_gps_node`: 桌面测试用 RTK/GPS mock。
- `demo_polygon_node`: 仅 `area_source:=demo` 时启动，用于 smoke test。

## Patrol Area Selector

小范围测试时推荐使用本地 Web 选区工具：

```text
启动 patrol_area_selector
-> 浏览器真实地图手动画区域
-> Save & Generate
-> data/input/patrol_area.json
-> static obstacle buffers
-> flyable area
-> UAV coverage route
-> DJI mission/KMZ
```

字段顺序约定：

- 前端 Leaflet 使用 `lat / lng`。
- GeoJSON 通常使用 `longitude / latitude`。
- 本项目保存的 JSON 统一使用 `latitude / longitude`。

## DJI App / MSDK

App 端执行流程：

```text
读取 data/output/dji_mission.kmz
-> pushKMZFileToAircraft(kmzPath)
-> 等待上传完成
-> startMission(missionFileName)
-> 监听任务执行状态
-> 监听当前航线 ID 和当前航点序号
-> 任务完成后按 finish_action 处理
```

## Legacy

旧 `sanitation_*`、`depth_anything_v3`、`sanitation_navigation_cpp` 包仍保留，避免破坏历史代码和 build；当前 UAV patrol 主线不从这些包启动。
