# 局部避障链路

默认链路：

`mock_camera -> DA3 TensorRT -> PointCloud2 -> ground_filter_node`

`-> local_costmap_builder_node -> local_astar_planner_node`

主要话题：

- `/depth_anything_v3/output/depth_image`
- `/depth_anything_v3/output/point_cloud`
- `/depth_anything/points_filtered`
- `/local_occupancy_grid`
- `/local_path`

`ground_filter_node` 负责 optical 到标准相机坐标转换、盲区与地面过滤。
`local_costmap_builder_node` 负责栅格内点数统计和障碍物膨胀。
`local_astar_planner_node` 在二值障碍栅格上生成距离衰减代价场，使用 weighted A* 提前选择
左右空闲通道，并通过平滑性和上一帧路径代价减少贴边与路径抖动。两侧均无法到达目标时，
`/planning_status` 发布 `STOPPED_BOTH_SIDES_BLOCKED`，同时 `/local_path` 发布空路径。

一键启动：

```bash
bash scripts/run_da3_video.sh
```

真实部署时应替换手机视频近似内参，并标定相机到 `base_link` 的外参。
