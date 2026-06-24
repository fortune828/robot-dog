# Robot-Dog / UAV ROS2

环卫机器人 ROS2 工作区。项目早期面向四足机器人，目前开始转向无人机实测：优先用
OSM 离线地图和覆盖路径规划生成离散 GPS 航点，供 DJI Matrice 4E 及学长已有 App 使用。

无人机主链路：

```text
DJI RTK /fix
-> 第一帧 RTK 锁定本次 ENU 原点
-> OSM 离线路由 / 多边形覆盖规划
-> /global_plan
-> uav_waypoint_exporter_node
-> data/output/uav_waypoints.csv + data/output/uav_waypoints.json
```

说明：

- 经纬度输出默认是 `WGS84`。
- 当前 CSV/JSON 是 Waypoint 3.0 兼容的中间格式；等拿到学长 App 或 DJI Pilot 2/SDK
  的实际导入样例后，再生成最终 WPML/KMZ 或接口格式。
- DJI Matrice 4E 默认使用相对起飞点高度，也可切换为海拔高度字段。
- Waypoint 3.0 单航线任务按最少 2 个、最多 65535 个航点约束处理。
- 坐标系默认 `WGS84`，参数可标记为 `CGCS2000`。
- 高度、速度、航点抽稀距离都在参数里配置。
- 机器狗 DA3 局部避障链路仍保留，作为后续地面平台或视觉避障研究分支。

## 无人机航点导出

构建后启动：

```bash
cd /home/ubuntu/bl/workspace/robot-dog
source setup_env.sh

ros2 launch sanitation_bringup uav_waypoint_export.launch.py \
  altitude_m:=30.0 \
  altitude_mode:=relative_to_takeoff \
  speed_mps:=5.0
```

如果暂时没有真实 DJI RTK `/fix`，可以用 mock GPS 做桌面验证：

```bash
ros2 launch sanitation_bringup uav_waypoint_export.launch.py use_mock_gps:=true
```

生成路径后会输出：

```text
data/output/uav_waypoints.csv
data/output/uav_waypoints.json
```

生成 DJI Waypoint 3.0 / WPML-KMZ 航线任务文件：

```bash
python scripts/uav_waypoints_to_dji_mission_converter.py
```

输出：

```text
data/output/dji_mission_draft.json
data/output/dji_mission.kmz
data/output/dji_mission_validation.log
```

DJI App / MSDK 对接说明见 `DJI_INTEGRATION_NOTES.md`。

关键参数在 `src/sanitation_bringup/config/default_params.yaml`：

```yaml
uav_waypoint_exporter_node:
  ros__parameters:
    origin_mode: "first_fix"
    altitude_m: 30.0
    altitude_mode: "relative_to_takeoff"
    speed_mps: 5.0
    min_spacing_m: 2.0
    min_waypoints: 2
    max_waypoints: 65535
    coordinate_frame: "WGS84"
    protocol: "DJI_WAYPOINT_3_0"
    aircraft_model: "DJI Matrice 4E"
```

## 机器狗局部避障链路

保留的默认局部感知规划链路：

```text
mock camera 30Hz
-> DA3 TensorRT FP16 component, max 10Hz inference
-> GPU BEV adaptive ground filter
-> filtered PointCloud2
-> C++ local costmap component
-> Python local A*
-> /local_path
```

- DA3 模型和 engine 位于 `models/da3/`。
- 测试视频位于 `data/videos/`。
- 默认使用 `CUDA_VISIBLE_DEVICES=1`，即第二张 GPU。
- 默认关闭 raw PointCloud2 和深度 debug 图，减少额外开销。
- `depth_anything_v3` 和 C++ costmap 以 ROS2 Components 形式运行在同一个 `component_container_mt` 中，并启用 intra-process communication。
- 旧 `ground_filter_node` 仍保留为 debug/回退链路，但默认不走。

## 构建

推荐使用统一脚本：

```bash
cd /home/ubuntu/bl/workspace/robot-dog
bash rebuild.sh
```

如果只更新了 DA3/C++ 热路径，可单独构建：

```bash
conda activate robotdog
source /opt/ros/humble/setup.bash
cd /home/ubuntu/bl/workspace/robot-dog

colcon build --symlink-install \
  --packages-select depth_anything_v3 sanitation_navigation_cpp sanitation_navigation sanitation_bringup \
  --cmake-args \
    -DDEPTH_ANYTHING_V3_ENABLE_CUDA_KERNELS=ON \
    -DPython3_EXECUTABLE=/usr/bin/python3
```

## 启动机器狗视频避障

视频避障启动：

```bash
conda activate robotdog
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sanitation_bringup da3_video_experiment.launch.py
```

常用参数：

```bash
ros2 launch sanitation_bringup da3_video_experiment.launch.py \
  video_path:=/home/ubuntu/bl/workspace/robot-dog/data/videos/test_video.mp4 \
  camera_publish_rate:=30.0 \
  max_inference_rate:=10.0
```

取消 DA3 限频：

```bash
ros2 launch sanitation_bringup da3_video_experiment.launch.py max_inference_rate:=0.0
```

使用旧 raw PointCloud2 + CPU ground filter debug 链路：

```bash
ros2 launch sanitation_bringup da3_video_experiment.launch.py \
  use_gpu_ground_filter:=false \
  enable_point_cloud:=true
```

## 性能测试

另开终端：

```bash
conda activate robotdog
source /opt/ros/humble/setup.bash
source install/setup.bash

python scripts/profile_ros_pipeline.py --frames 100
```

单独测试 DA3 engine：

```bash
python scripts/benchmark_da3_engine.py \
  --video data/videos/test_video.mp4 \
  --engine models/da3/DA3METRIC-LARGE.fp16-batch1.engine
```

## 当前关键参数

默认视频配置在 `src/sanitation_bringup/config/da3_video_params.yaml`：

```yaml
mock_camera_node:
  ros__parameters:
    publish_rate: 30.0

depth_anything_v3:
  ros__parameters:
    max_inference_rate: 10.0
    enable_gpu_ground_filter: true
    enable_point_cloud: false
```

局部路径保持配置在 `src/sanitation_bringup/config/local_avoidance.yaml`：

```yaml
local_astar_planner_node:
  ros__parameters:
    hold_last_path_enabled: true
    hold_publish_rate: 10.0
```

## 简单检查

```bash
python -m py_compile src/sanitation_navigation/sanitation_navigation/local_astar_planner_node.py
colcon build --packages-select depth_anything_v3 sanitation_navigation_cpp sanitation_navigation sanitation_bringup \
  --cmake-args -DDEPTH_ANYTHING_V3_ENABLE_CUDA_KERNELS=ON -DPython3_EXECUTABLE=/usr/bin/python3
```
