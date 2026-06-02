# AGENTS.md

本文件是《四足环卫机器人具身智能系统》项目的全局开发规范、架构基准与 AI 代理指令集。所有后续的代码生成、重构和模块扩展，必须严格遵循本文件的约定。

## 一、 项目现状与里程碑总结 (已达成)

当前系统已完成 **"底层底盘控制 + 宏观全局导航 + 单目深度感知"** 三大基座的闭环开发，并建立了完整的测试框架。

**核心已达成能力：**

1. **数字孪生与离线路网基座 (`osm_map_manager`)**：
   - 彻底废除在线 API，实现本地 OpenStreetMap (OSM) 矢量拓扑地图的解析与有向图构建（13000+ 节点）。
   - 提供 `/plan_osm_path` 离线路由服务，解决穿墙问题，实现贴合路网的最短路径计算。
   - 在 RViz2 中完美实现校园数字孪生 3D 拓扑的可视化高亮广播（路网 + 建筑轮廓）。

2. **业务逻辑状态机与全覆盖规划 (`polygon_coverage_planner`)**：
   - 实现优雅的人机交互（RViz2 点选闭合多边形）。
   - 基于 `Shapely` 的最长边寻优扫描线（弓字形）填充算法。
   - **双轨拼接与折返逻辑**：自动寻找最近接入点，拼接"OSM通勤段"与"区域覆盖段"，并通过事件驱动机制（监听 `/mission_status`）实现自动逆序数组的无限折返跑。
   - 具备高优先级任务（如手动 P2P 导航）随时跨节点打断当前巡逻状态的能力。

3. **极简且忠诚的小脑控制 (`mock_gps_node`)**：
   - 维持零业务耦合，专注消费 `/global_plan` 话题，以精准步进执行点到点直线追踪。
   - 到达终点时发布 `REACHED_GOAL` 事件，驱动上层业务状态机。
   - 空闲时在原点附近发布带噪声的静止 GPS 信号，并持续广播 `world → odom` TF（含 Yaw）。

4. **单目深度感知链路 (`sanitation_perception`)** — Phase 1 已完成：
   - `depth_to_scan_node`：集成 Depth Anything V2 (vits) 本地模型，实时推理 MP4 视频 → 发布 32FC1 深度图 (`/camera/depth_image`) + 虚拟 CameraInfo (`/camera/camera_info`)。
   - `depth_to_cloud_node`：深度图 → 针孔反投影 → 3D 点云 (`/camera/depth_points`, PointCloud2)，支持 stride 下采样控制点密度。
   - 遵循 ROS 2 Iron 铁律：单帧单时间戳、统一 frame_id、真实 32FC1 公制深度 [0.5, 10.0]m。

5. **目标检测链路 (Mock 模式)** — Phase 2 雏形：
   - `mock_camera_node`：模拟摄像头，优先读取 `data/test_video.mp4`，无视频时生成合成图像，15Hz 发布 `/camera/image_raw`。
   - `detection_node`：订阅 `/camera/image_raw`，基于 `sanitation_core.detection_utils` 的 Mock YOLO 逻辑生成模拟检测结果，发布 `/detection/annotated_image` + `/detection/results` (JSON)。

6. **辅助节点与工具链**：
   - `mock_chassis_node`：模拟底盘运动学，订阅 `/cmd_vel`，发布 `/odom` + TF (`odom → base_link → gps_link`)。
   - `waypoint_patrol_node`：路点巡检控制，基于 `sanitation_core.navigation_utils` 的纯追踪控制器发布 `/cmd_vel`。
   - `gaode_path_proxy`：高德步行路径规划代理（GCJ-02 ↔ WGS-84 双向转换 + polyline 解码 → `/global_plan`）。
   - `sanitation_core`：纯 Python 算法库，含 `detection_utils`（Mock YOLO）和 `navigation_utils`（Pure Pursuit、路点管理）。

7. **测试框架**：
   - `tests/` 目录，pytest 驱动，`conftest.py` 自动注入 `src/` 路径。
   - `test_detection_utils.py`：7 个单元测试（空检测、确定性种子、bbox 边界、类别合法性等）。
   - `test_navigation_utils.py`：10+ 个单元测试（角度归一化、纯追踪方向/限幅/减速、路点切换/循环等）。
   - 无 ROS 环境可直接运行。

8. **启动与配置**：
   - `demo_mock_system.launch.py`：一键启动 6 节点全 Mock 闭环（camera + detection + chassis + gps + osm + cpp）。
   - `perception.launch.py`：Phase 1 独立启动（depth_to_cloud_node + 可调参数）。
   - `default_params.yaml`：全局默认参数集中管理，切换 Mock/Real 模式无需改代码。

## 二、 架构原则 (核心红线)

1. **基于 ROS 2 的节点化解耦 (rclpy)：**
   - 宏观导航大脑（Planner）、底层执行小脑（Tracker）、视觉感知（Perception）必须是完全独立的节点。
   - 绝对禁止节点间的硬编码强耦合，所有通信必须且只能通过 Topic / Service 交互。

2. **硬件抽象与 Mock 优先 (HAL)：**
   - 业务节点不能直接写死传感器或硬件 SDK。必须使用标准化消息通信（如 `sensor_msgs/Image`，`nav_msgs/Path`）。
   - 所有节点应支持无硬件环境下的 Mock 降级运行。

3. **"大脑极度聪明，小脑保持愚蠢"：**
   - 底层控制节点只管执行轨迹和汇报状态，不参与业务逻辑决策；复杂的拼接、记忆、状态流转全由上层 Planner 节点通过内存变量与回调函数管理。

4. **纯函数与算法库分离：**
   - 核心算法逻辑应抽取到 `sanitation_core` 中作为纯 Python 函数（无 ROS 依赖），便于单元测试和独立沙盒验证。

## 三、 目录结构规范

请继续维护以下标准的 ROS 2 Python 包结构：

- `src/`
  - `sanitation_interfaces/`：自定义通信接口定义包（含 `.srv`, `.msg`）。
  - `sanitation_core/`：核心算法库（纯 Python 脚本，独立沙盒测试区，零 ROS 依赖）。
  - `sanitation_perception/`：视觉感知大脑包（含深度估计、点云生成、目标检测节点）。
  - `sanitation_navigation/`：宏观调度大脑包（含 `osm_map_manager`, `polygon_coverage_planner`, `mock_gps_node`, `mock_chassis_node`, `waypoint_patrol_node`, `gaode_path_proxy`）。
  - `sanitation_bringup/`：系统启动与参数中心（含 `.launch.py`, `.yaml`）。
- `data/`：本地数据区（存放 `map.osm`、MP4 测试视频、Depth Anything V2 模型代码与权重文件）。
- `tests/`：pytest 测试目录（`conftest.py` + 各模块单元测试）。

## 四、 当前进展：视觉纪元 Phase 1 完成，Phase 2 Mock 就绪

**已完成 — Phase 1：求生本能 (单目深度估计)**
- Depth Anything V2 (vits) 本地推理已集成，无需 HuggingFace / Transformers 重型依赖。
- 两条输出路径并行：`depth_to_scan_node`（2D 深度图 + CameraInfo）和 `depth_to_cloud_node`（3D 点云）。
- 模型权重 (`depth_anything_v2_vits.pth`) 和代码 (`dinov2.py`, `dpt.py`) 已就绪于 `data/` 目录。
- 架构师已验证推理帧率与边缘分割精度。

**已完成 — Phase 2 雏形：干活本能 (YOLO 目标检测 Mock)**
- `detection_node` 已实现完整的检测管线（订阅 → 推理 → 标注 → 发布），当前使用 Mock YOLO 逻辑。
- `sanitation_core.detection_utils` 提供 Mock 检测生成与绘制函数，可无缝替换为真实 YOLO 模型调用。

**下一步 — Phase 2 生产化：真实 YOLO 集成**
- 将 `detection_node` 中的 Mock YOLO 替换为 ultralytics YOLO 模型推理。
- 结合当前机器狗姿态、RTK 经纬度与深度物理距离，在 RViz2 中实时抛出垃圾锚点。
- 下载轻量化 YOLO 权重文件到 `data/` 目录。

**规划中 — Phase 3：认知建构 (语义体素地图)**
- 融合几何深度特征与语义标签，动态构建 Semantic Voxel Map。
- 在云端或本地算力池中实现 3D 语义理解。

## 五、 当前开发指令

**当前焦点**：将 Phase 2 从 Mock 模式升级为真实 YOLO 推理。

具体待办：
1. 在 `sanitation_core/detection_utils.py` 中添加真实 YOLO 推理函数（或创建独立的推理模块）。
2. 修改 `detection_node` 支持通过参数切换 Mock / Real 模式。
3. 利用 `depth_to_cloud_node` 输出的点云或 `depth_to_scan_node` 输出的深度图，将检测到的目标的像素坐标映射为 3D 世界坐标（需要深度值 + 相机内参）。
4. 在 RViz2 中发布 3D 垃圾锚点 Marker。

在修改任何现有 ROS 2 框架节点前，请先在 `sanitation_core` 中完成纯 Python 沙盒验证。
