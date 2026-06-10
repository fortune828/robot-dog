# Robot Dog ROS2

默认局部感知规划链路：

`mock camera -> DA3 TensorRT -> PointCloud2 -> ground filter -> local costmap -> local A*`

模型位于 `models/da3/`，测试视频位于 `data/videos/`，生成结果放入
`data/output/`。默认启动使用物理第二张 GPU，并关闭深度彩色调试图以减少额外开销。

DA3 TensorRT ROS2 wrapper 当前放在同级目录
`/home/ubuntu/bl/workspace/Depth/ros2-depth-anything-v3-trt`，首次使用前运行准备脚本完成编译与模型链接。

## 构建与启动

```bash
conda activate robotdog
cd /home/ubuntu/bl/workspace/robot-dog
bash scripts/prepare_da3.sh
colcon build --symlink-install
bash scripts/run_da3_video.sh
```

第二个终端查看全链路性能：

```bash
conda activate robotdog
source /opt/ros/humble/setup.bash
source install/setup.bash
python scripts/profile_ros_pipeline.py --frames 100
```

测试：

```bash
pytest -q
colcon build --symlink-install
```
