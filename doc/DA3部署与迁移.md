# DA3 视频实验与真实设备迁移

## 当前方案

服务器实验使用 `DA3METRIC-LARGE`、FP16 TensorRT、物理第二张 GPU。运行进程设置
`CUDA_VISIBLE_DEVICES=1`，因此该进程内部看到的 CUDA 设备编号是 `0`。

数据链路保持真实部署接口：

`/camera/image_raw` + `/camera/camera_info` -> DA3 -> PointCloud2 -> 地面滤波 -> 局部栅格 -> A*

DA3 wrapper 发布 optical 坐标点云（X 右、Y 下、Z 前）。`ground_filter_node` 用
`input_convention: optical` 转成 ROS 标准相机坐标（X 前、Y 左、Z 上），输出 frame 为
`camera_link`。

## iPhone 视频限制

当前 1280x720 视频使用 `focal_length_px: 960.0`，只是便于验证尺度与链路的近似值，
不是 iPhone 16 Pro Max 的标定结果。单目 metric depth 的尺度仍受镜头、裁剪、视频防抖和场景影响。
判断模型效果时应同时观察深度图连续性、近距离障碍召回和点云稳定性，不只看绝对米制误差。

## 运行

```bash
./scripts/prepare_da3.sh
./scripts/run_da3_video.sh [视频路径] [近似焦距像素]
```

查看输出：

```bash
ros2 topic hz /depth_anything_v3/output/depth_image
ros2 topic echo /depth_anything_v3/output/point_cloud --once
ros2 topic echo /local_path --once
```

## 迁移到真实机器人

1. 用真实相机驱动替换 `mock_camera_node`，继续发布同名 Image 和同步 CameraInfo。
2. 使用标定得到的 K/P、畸变参数和 optical frame，不再使用近似焦距。
3. 按实际安装位置修改 `camera_x/y/z` 与 roll/pitch/yaw，并校验 TF。
4. 在目标 GPU、目标 TensorRT 版本上重新生成 engine。TensorRT engine 不应跨 GPU 架构或版本复制。
5. 根据端侧算力调整输入分辨率、点云下采样和发布频率；下游话题与坐标约定无需改变。

当前 wrapper 原生面向 TensorRT，适合服务器和 NVIDIA Jetson 路线。若最终端侧不是 NVIDIA，保留
ROS 接口并替换 DA3 推理后端即可，地面过滤与导航链路不需要重写。

## C++ 主链路与 Python 独立脚本

ROS2 主链路使用 `src/depth_anything_v3/` 中的 C++ TensorRT package。它执行 RGB 转换、ImageNet mean/std 归一化、metric focal scaling、sky mask、深度缩放和 PointCloud2 生成。

`scripts/benchmark_da3_engine.py` 与 C++ 使用相同的 RGB 和 ImageNet mean/std 输入归一化，但直接可视化原始 depth tensor，不执行 metric focal scaling、sky mask 和 PointCloud2 后处理。因此其 `Pure-engine FPS` 用于评估 TensorRT engine 推理速度，视觉结果和完整链路 FPS 不应与 C++ ROS2 主链路直接比较。
