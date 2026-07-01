# UAV Planning Logic

当前 UAV patrol 主线只做区域覆盖航线生成，不做实时绕障和复杂世界模型。

## Default Flow

```text
data/input/patrol_area.json
data/map.osm building=*
-> target_area / hard_obstacles / coverage_target_area / planning_airspace
-> semantic_coverage_planner
-> /global_plan
-> uav_waypoint_exporter_node
-> data/output/uav_waypoints.json/csv
-> data/output/dji_mission_draft.json
-> data/output/dji_mission.kmz
```

## Patrol Area Input

默认输入文件是 `data/input/patrol_area.json`：

```json
{
  "area_name": "custom_patrol_area",
  "coordinate_frame": "WGS84",
  "boundary": [
    {"latitude": 30.0, "longitude": 103.0},
    {"latitude": 30.0, "longitude": 103.0},
    {"latitude": 30.0, "longitude": 103.0}
  ]
}
```

小范围测试推荐使用网页选区工具生成该文件：

```bash
python scripts/patrol_area_selector_server.py
```

启动后打开 `http://localhost:8000`，在真实地图底图上画 polygon，点击 `Save` 后会自动写入 `data/input/patrol_area.json`。

点击 `Save & Generate` 后会自动执行语义覆盖规划，生成 `uav_waypoints.json/csv`、`target_area.geojson`、`coverage_target_area.geojson`、`planning_airspace.geojson`、`static_obstacles.geojson`、`static_obstacle_buffers.geojson`、DJI mission/KMZ 和 `patrol_route_visualization.html`。网页选区小范围测试不需要再手动运行 ROS launch。

## OSM Role

`data/map.osm` 当前只作为地图底图范围、巡逻区域来源和后续语义风险先验，不作为无人机道路导航来源。UAV 默认不调用 `/plan_osm_path`，不沿 OSM road graph 飞行。

当前静态语义约束：

```text
patrol_area.json
+ data/map.osm building=*
-> static_obstacles.geojson
-> static_obstacle_buffers.geojson
-> coverage_target_area.geojson
-> planning_airspace.geojson
-> semantic constrained coverage route
```

默认策略：

- 建筑物区域不飞越；
- 建筑物外扩 `building_safety_buffer_m: 5.0`；
- OSM 经纬度先转局部米制坐标，再做 buffer/difference；
- 小于 `min_flyable_area_m2: 5.0` 的碎片区域会丢弃；
- 长直线扫掠段只保留必要端点，避免 DJI 航点过密；
- 用户画的 polygon 是 `target_area`，同时也是当前静态规划的飞行硬边界；
- 建筑物安全 buffer 是 `hard_obstacles`，是真正硬约束；
- `planning_airspace = target_area - hard_obstacles`；
- 默认不允许目标区外通勤，`max_off_area_distance_m: 0.0`；
- 候选 sweep stroke 在 `planning_airspace` 内生成，但必须对 `coverage_target_area` 有覆盖贡献；
- 主体扫线和连接段都限制在用户画的 polygon 内；
- 边界观察线会拆成线段参与统一排序，避免最后追加大环线造成大量交叉；
- 连接段会做最终检查，穿过 building buffer 的连接会被重新绕行或丢弃。
- 不做动态障碍、视觉预测、在线绕障和飞控控制。

无 ROS 快速验证：

```bash
python scripts/run_semantic_coverage_planning.py
```

## Route Visualization

生成航点后可重新生成 HTML 可视化：

```bash
python scripts/visualize_patrol_area_and_route.py
```

输出：

```text
data/output/patrol_route_visualization.html
```

HTML 中会显示 target area、planning airspace、coverage target area、OSM building、building safety buffer、coverage route 和航点编号；默认不会产生目标区外通勤段。
