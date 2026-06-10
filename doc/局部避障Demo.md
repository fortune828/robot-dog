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

一键启动：

```bash
bash scripts/run_da3_video.sh
```

真实部署时应替换手机视频近似内参，并标定相机到 `base_link` 的外参。
