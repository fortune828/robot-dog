# Handoff For Next Agent

本项目已从早期 `robot-dog / sanitation ground robot` 方向，切换到：

```text
UAV patrol / 无人机自主巡逻
```

当前阶段只做：

```text
用户在地图上画巡逻区域
-> 基于 OSM 建筑物语义生成静态障碍 buffer
-> 扣除不可飞区域
-> 在剩余可飞区域生成覆盖巡逻航线
-> 输出 UAV waypoint JSON/CSV + DJI WPML/KMZ + 网页可视化
```

当前不要做：

```text
动态障碍
视觉目标检测
在线绕障
真实飞控底层控制
Android / DJI App 对接
```

## Current Main Workflow

推荐从网页工具启动，不需要手动跑 ROS launch：

```bash
cd /home/ubuntu/bl/workspace/robot-dog
source setup_env.sh
python scripts/patrol_area_selector_server.py
```

打开服务器桌面浏览器：

```text
http://localhost:8000
```

网页操作：

```text
1. 在真实地图上画 polygon
2. 点击 Save & Generate
3. 后端自动保存 data/input/patrol_area.json
4. 后端自动运行 semantic coverage planning
5. 当前网页地图直接叠加显示：
   - 用户画的 patrol area
   - OSM building
   - building safety buffer
   - flyable area
   - UAV route
   - sampled waypoint labels
```

如果从非服务器本机浏览器访问，需要用：

```bash
python scripts/patrol_area_selector_server.py --host 0.0.0.0
```

然后访问：

```text
http://服务器IP:8000
```

注意：网页依赖在线 OSM tile 和 Leaflet/Leaflet-Draw CDN。之前排查过，如果 `unpkg.com` CDN 在服务器网络里 SSL 握手超时，页面可能空白；这不是 Python server 崩了。Python server 本机 `http://127.0.0.1:8000/` 和 `/api/config` 返回 200。

## Important Files

```text
README.md
UAV_PLANNING_LOGIC.md
HANDOFF_FOR_NEXT_AGENT.md

data/map.osm
data/input/patrol_area.json
data/output/

scripts/patrol_area_selector_server.py
scripts/run_semantic_coverage_planning.py
scripts/visualize_patrol_area_and_route.py

web/patrol_area_selector.html

src/uavpatrol_navigation/uavpatrol_navigation/semantic_coverage_planner.py
src/uavpatrol_navigation/uavpatrol_navigation/polygon_coverage_planner.py
src/uavpatrol_navigation/uavpatrol_navigation/uav_waypoint_exporter_node.py
src/uavpatrol_navigation/uavpatrol_navigation/patrol_visualization.py
src/uavpatrol_navigation/uavpatrol_navigation/patrol_area_io.py

src/uavpatrol_bringup/config/default_params.yaml
src/uavpatrol_bringup/launch/uav_waypoint_export.launch.py
```

旧包仍存在但现在是 legacy：

```text
sanitation_*
depth_anything_v3
sanitation_navigation_cpp
```

不要轻易删除旧包，因为 `colcon build` 仍会构建 9 个包。

## Current Default Parameters

网页 selector server 默认值已经内置，不需要用户每次输一长串参数。
唯一默认参数入口是：

```text
src/uavpatrol_navigation/uavpatrol_navigation/planning_defaults.py
```

其他脚本、网页 server、ROS node、launch 只引用 / 透传这些默认值：

```text
map_file: data/map.osm
patrol_area_file: data/input/patrol_area.json
route_output: data/output
coverage_spacing_m: 10.0
building_safety_buffer_m: 5.0
default_building_height_m: 20.0
min_flyable_area_m2: 5.0
route_edge_margin_m: 3.0
max_off_area_distance_m: 0.0
sensor_coverage_radius_m: 0.0
min_coverage_contribution_m: 1.0
max_connection_length_m: 150.0
simplify_tolerance_m: 0.5
max_2opt_iterations: 200
disable_2opt_if_strokes_gt: 300
altitude_m: 30.0
speed_mps: 5.0
```

含义：

```text
coverage_spacing_m:
  默认扫掠间距。刚从 5m 调到 10m，原因是航点太密。

building_safety_buffer_m:
  建筑物外扩安全距离。当前 5m。

min_flyable_area_m2:
  小碎片过滤阈值。当前 5 平方米，几乎只要有面积就扫。

route_edge_margin_m:
  路线优先离边界 / buffer 内缩 3m，避免一直贴边走。

max_off_area_distance_m:
  当前回退为 0m。用户画的 polygon 同时是覆盖目标区和静态规划飞行硬边界。
  连接 / 通勤段默认不允许飞出 target_area。

sensor_coverage_radius_m:
  第一版默认 0m，表示路径本身覆盖；后续可调 10m / 20m 模拟相机视场。
```

如果用户觉得航线仍太密，可建议：

```bash
python scripts/patrol_area_selector_server.py --coverage-spacing 12.0
```

当前同一区域测试过：

```text
10m -> 约 362 个点
12m -> 约 277 个点
15m -> 约 201 个点
```

如果用户觉得航线太靠边，可建议：

```bash
python scripts/patrol_area_selector_server.py --route-edge-margin 4.0
```

如果窄通道扫不到，再降到：

```bash
--route-edge-margin 2.0
```

## Semantic Planning Logic

核心文件：

```text
src/uavpatrol_navigation/uavpatrol_navigation/semantic_coverage_planner.py
```

核心流程：

```text
patrol_area.json WGS84 polygon
-> 转 local ENU / metric
-> 只从 target_area.buffer(building_safety_buffer_m) 上下文提取 building=*
-> building polygon buffer(5m)
-> target_area = 用户 polygon
-> hard_obstacles = union(building_buffers)
-> coverage_target_area = target_area - hard_obstacles
-> planning_airspace = target_area - hard_obstacles
-> 删除小于 min_flyable_area_m2 的 coverage_target_area 碎片
-> 在 planning_airspace 内生成候选 stroke
-> 只保留对 coverage_target_area.buffer(sensor_coverage_radius_m) 有覆盖贡献的 stroke
-> 连接 / 通勤段必须留在 planning_airspace 内，默认不出用户画的 polygon
-> 扫线和边界观察线拆成 stroke 后统一做最近安全排序，减少无意义交叉
-> 2-opt 后处理继续消除安全可改的自交叉
-> 最终逐段检查，任何穿过 building buffer 的连接会重算或丢弃
```

重要设计变化：

```text
早期版本太保守，只扫到右下角少量航线。
后来改成先收集所有可飞扫线段，再尽量串联。
再后来为避免贴边，改成优先在内缩区域生成航线。
最后将默认密度从 5m 放宽到 10m，避免航点过多。
```

当前几何安全检查曾跑过：

```text
route_points: 362
violating_segments: 0
```

`violating_segments` 是指航段穿过 building buffer 的数量。

## Outputs

网页 `Save & Generate` 后会生成：

```text
data/input/patrol_area.json

data/output/uav_waypoints.json
data/output/uav_waypoints.csv
data/output/static_obstacles.geojson
data/output/static_obstacle_buffers.geojson
data/output/target_area.geojson
data/output/coverage_target_area.geojson
data/output/planning_airspace.geojson
data/output/transit_segments.geojson
data/output/flyable_area.geojson  # compatibility alias, not the core model
data/output/dji_mission_draft.json
data/output/dji_mission.kmz
data/output/dji_mission_validation.log
data/output/patrol_route_visualization.html
```

不再保留重复输出：

```text
semantic_uav_waypoints.json
semantic_uav_waypoints.csv
```

这些旧重复文件如果出现，通常是历史残留；selector server 保存新区域时会清理它们。

## DJI Output

转换逻辑在：

```text
src/uavpatrol_navigation/uavpatrol_navigation/uav_waypoints_to_dji_mission_converter.py
```

KMZ 内含：

```text
wpmz/template.kml
wpmz/waylines.wpml
```

当前仍然只是生成文件，不直接控制无人机。后续 DJI Pilot 2 / MSDK App 上传执行由另一个系统处理。

## ROS Mainline

虽然网页工具已经能不跑 ROS 完成主流程，但 ROS 主线仍保留：

```bash
source setup_env.sh
ros2 launch uavpatrol_bringup uav_waypoint_export.launch.py use_mock_gps:=true
```

ROS 主线默认也会读取：

```text
data/input/patrol_area.json
data/map.osm
```

并使用语义约束生成 `/global_plan`，再由 exporter 输出 UAV waypoints 和 DJI files。

## Common User Expectations

用户明确偏好：

```text
1. 不要让用户每次输一长串参数；
2. 默认值写进代码 / config；
3. README 里只放最常用短命令；
4. 网页画完区域后点击 Save & Generate，应直接在原地图上显示航线；
5. 不要要求用户再手动跑 ros2 launch 才看结果；
6. 航线不要太贴边，正常区域应偏中间；
7. 但窄缝、走廊也要穿过去，因为穿过本身也是观察；
8. 不追求严格弓字型，目标是巡逻区域都走过、看过；
9. 输出文件不要太多，只保留后续可用和人能看懂的。
```

## Known Issues / Watchpoints

1. Leaflet CDN 网络问题

   `web/patrol_area_selector.html` 当前引用：

   ```text
   https://unpkg.com/leaflet@1.9.4/dist/leaflet.css
   https://unpkg.com/leaflet-draw@1.0.4/dist/leaflet.draw.css
   https://unpkg.com/leaflet@1.9.4/dist/leaflet.js
   https://unpkg.com/leaflet-draw@1.0.4/dist/leaflet.draw.js
   ```

   如果网页空白，先查浏览器 DevTools 网络请求。之前服务器上 `unpkg.com` SSL 握手超时过，但 `tile.openstreetmap.org` 是通的。

2. 输出为空或航点很少

   先检查画的区域是否几乎完全被 building buffer 吃掉：

   ```text
   data/output/coverage_target_area.geojson
   data/output/planning_airspace.geojson
   data/output/static_obstacle_buffers.geojson
   ```

   可临时降低：

   ```bash
   --building-buffer 3.0
   --route-edge-margin 2.0
   ```

3. 航点太密

   当前默认 `coverage_spacing=10.0`。可加大：

   ```bash
   --coverage-spacing 12.0
   --coverage-spacing 15.0
   ```

4. 航线太靠边

   当前默认 `route_edge_margin=3.0`。可加大：

   ```bash
   --route-edge-margin 4.0
   ```

5. 必须先 source 环境

   `shapely` 等依赖在 `robotdog` conda 环境里。直接系统 Python 跑脚本会出现：

   ```text
   ModuleNotFoundError: No module named 'shapely'
   ```

   正确：

   ```bash
   cd /home/ubuntu/bl/workspace/robot-dog
   source setup_env.sh
   python scripts/patrol_area_selector_server.py
   ```

## Validation Commands

常用快速检查：

```bash
cd /home/ubuntu/bl/workspace/robot-dog
source setup_env.sh

python3 -m py_compile \
  scripts/patrol_area_selector_server.py \
  scripts/run_semantic_coverage_planning.py \
  scripts/visualize_patrol_area_and_route.py \
  src/uavpatrol_navigation/uavpatrol_navigation/*.py

pytest -q
colcon build --symlink-install
```

最近一次验证结果：

```text
pytest: 43 passed
colcon build: 9 packages finished
```

几何安全检查示例：

```bash
source setup_env.sh
python3 - <<'PY'
import sys
sys.path.insert(0, 'src/uavpatrol_navigation')
from shapely.geometry import LineString
from shapely.ops import unary_union
from uavpatrol_navigation.semantic_coverage_planner import plan_semantic_coverage

res = plan_semantic_coverage(
    map_file='data/map.osm',
    patrol_area_file='data/input/patrol_area.json',
    building_safety_buffer_m=5.0,
    min_flyable_area_m2=5.0,
    coverage_spacing_m=10.0,
    route_edge_margin_m=3.0,
    write_outputs=False,
)
obs = unary_union(res.obstacle_buffers_local)
viol = sum(
    1
    for a, b in zip(res.route_local, res.route_local[1:])
    if (not obs.is_empty) and LineString([a, b]).intersects(obs)
)
print('route_points', len(res.route_local))
print('violating_segments', viol)
PY
```

## Current Suggested Answer If User Asks How To Start

```bash
cd /home/ubuntu/bl/workspace/robot-dog
source setup_env.sh
python scripts/patrol_area_selector_server.py
```

Then open:

```text
http://localhost:8000
```

Click:

```text
Save & Generate
```

The output route and semantic layers should appear directly on the same map.
