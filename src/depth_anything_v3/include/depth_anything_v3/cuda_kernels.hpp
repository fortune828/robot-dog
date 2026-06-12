#ifndef DEPTH_ANYTHING_V3__CUDA_KERNELS_HPP_
#define DEPTH_ANYTHING_V3__CUDA_KERNELS_HPP_

#include <cuda_runtime.h>
#include <cstdint>

namespace depth_anything_v3
{

cudaError_t launchBgrToNchwPreprocess(
  const uint8_t * input_bgr, int src_width, int src_height, int src_step_bytes,
  float * output_nchw, int dst_width, int dst_height, cudaStream_t stream);

cudaError_t launchDepthCleanScale(
  const float * depth, const float * sky, float * output_depth, int width, int height,
  float focal_scale, float sky_threshold, float sky_depth_cap, cudaStream_t stream);

cudaError_t launchDepthResizeBilinear(
  const float * input_depth, int input_width, int input_height, float * output_depth,
  int output_width, int output_height, cudaStream_t stream);

cudaError_t launchDepthToFilteredCloud(
  const float * depth, const float * sky, float * output_xyzi, unsigned int * output_count,
  int * bev_ground_min_z_mm, int width, int height, int downsample_factor, float fx, float fy,
  float cx, float cy,
  float camera_x, float camera_y, float camera_z, float cos_pitch, float sin_pitch,
  float blind_spot, float min_z, float max_z, float min_depth, float max_depth,
  float bev_resolution, float bev_height_diff, int bev_origin_x, int bev_origin_y,
  int bev_width, int bev_height, float sky_threshold, cudaStream_t stream);

}  // namespace depth_anything_v3

#endif  // DEPTH_ANYTHING_V3__CUDA_KERNELS_HPP_
