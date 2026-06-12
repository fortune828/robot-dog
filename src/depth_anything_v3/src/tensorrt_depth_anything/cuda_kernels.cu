#include "depth_anything_v3/cuda_kernels.hpp"

#include <algorithm>
#include <cmath>

namespace depth_anything_v3
{
namespace
{

__device__ float clampFloat(float value, float low, float high)
{
  return fminf(fmaxf(value, low), high);
}

__global__ void bgrToNchwPreprocessKernel(
  const uint8_t * input_bgr, int src_width, int src_height, int src_step_bytes,
  float * output_nchw, int dst_width, int dst_height)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= dst_width || y >= dst_height) {
    return;
  }

  const float src_x = (static_cast<float>(x) + 0.5f) * src_width / dst_width - 0.5f;
  const float src_y = (static_cast<float>(y) + 0.5f) * src_height / dst_height - 0.5f;
  const int x0 = static_cast<int>(floorf(clampFloat(src_x, 0.0f, static_cast<float>(src_width - 1))));
  const int y0 = static_cast<int>(floorf(clampFloat(src_y, 0.0f, static_cast<float>(src_height - 1))));
  const int x1 = min(x0 + 1, src_width - 1);
  const int y1 = min(y0 + 1, src_height - 1);
  const float wx = clampFloat(src_x - x0, 0.0f, 1.0f);
  const float wy = clampFloat(src_y - y0, 0.0f, 1.0f);

  const uint8_t * p00 = input_bgr + y0 * src_step_bytes + x0 * 3;
  const uint8_t * p01 = input_bgr + y0 * src_step_bytes + x1 * 3;
  const uint8_t * p10 = input_bgr + y1 * src_step_bytes + x0 * 3;
  const uint8_t * p11 = input_bgr + y1 * src_step_bytes + x1 * 3;

  const float mean[3] = {0.485f, 0.456f, 0.406f};
  const float std_vals[3] = {0.229f, 0.224f, 0.225f};
  const int pixel_index = y * dst_width + x;
  const int plane_size = dst_width * dst_height;

  for (int channel = 0; channel < 3; ++channel) {
    const int bgr_channel = 2 - channel;
    const float top =
      static_cast<float>(p00[bgr_channel]) * (1.0f - wx) +
      static_cast<float>(p01[bgr_channel]) * wx;
    const float bottom =
      static_cast<float>(p10[bgr_channel]) * (1.0f - wx) +
      static_cast<float>(p11[bgr_channel]) * wx;
    const float value = (top * (1.0f - wy) + bottom * wy) / 255.0f;
    output_nchw[channel * plane_size + pixel_index] = (value - mean[channel]) / std_vals[channel];
  }
}

__global__ void depthCleanScaleKernel(
  const float * depth, const float * sky, float * output_depth, int width, int height,
  float focal_scale, float sky_threshold, float sky_depth_cap)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height) {
    return;
  }

  const int index = y * width + x;
  float value = depth[index];
  if (!isfinite(value) || value <= 0.0f) {
    value = 0.0f;
  } else {
    value *= focal_scale;
  }

  if (sky && sky[index] >= sky_threshold) {
    value = sky_depth_cap;
  }

  output_depth[index] = value;
}

__global__ void depthResizeBilinearKernel(
  const float * input_depth, int input_width, int input_height, float * output_depth,
  int output_width, int output_height)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= output_width || y >= output_height) {
    return;
  }

  const float src_x = (static_cast<float>(x) + 0.5f) * input_width / output_width - 0.5f;
  const float src_y = (static_cast<float>(y) + 0.5f) * input_height / output_height - 0.5f;
  const int x0 = static_cast<int>(floorf(clampFloat(src_x, 0.0f, static_cast<float>(input_width - 1))));
  const int y0 = static_cast<int>(floorf(clampFloat(src_y, 0.0f, static_cast<float>(input_height - 1))));
  const int x1 = min(x0 + 1, input_width - 1);
  const int y1 = min(y0 + 1, input_height - 1);
  const float wx = clampFloat(src_x - x0, 0.0f, 1.0f);
  const float wy = clampFloat(src_y - y0, 0.0f, 1.0f);

  const float v00 = input_depth[y0 * input_width + x0];
  const float v01 = input_depth[y0 * input_width + x1];
  const float v10 = input_depth[y1 * input_width + x0];
  const float v11 = input_depth[y1 * input_width + x1];
  const float top = v00 * (1.0f - wx) + v01 * wx;
  const float bottom = v10 * (1.0f - wx) + v11 * wx;
  output_depth[y * output_width + x] = top * (1.0f - wy) + bottom * wy;
}

__device__ bool projectDepthToBase(
  const float * depth, const float * sky, int width, int height, int downsample_factor,
  float fx, float fy, float cx, float cy,
  float camera_x, float camera_y, float camera_z, float cos_pitch, float sin_pitch,
  float blind_spot, float min_depth, float max_depth,
  float sky_threshold, int out_x, int out_y, float & x_base, float & y_base, float & z_base,
  float & z_optical)
{
  const int u = out_x * downsample_factor;
  const int v = out_y * downsample_factor;
  if (u >= width || v >= height) {
    return false;
  }

  const int index = v * width + u;
  if (sky && sky[index] >= sky_threshold) {
    return false;
  }

  z_optical = depth[index];
  if (!isfinite(z_optical) || z_optical < min_depth || z_optical > max_depth) {
    return false;
  }

  const float x_optical = (static_cast<float>(u) - cx) * z_optical / fx;
  const float y_optical = (static_cast<float>(v) - cy) * z_optical / fy;

  // ROS optical frame: X right, Y down, Z forward.
  // Base frame convention: X forward, Y left, Z up.
  const float x_cam = z_optical;
  const float y_cam = -x_optical;
  const float z_cam = -y_optical;

  x_base = cos_pitch * x_cam + sin_pitch * z_cam + camera_x;
  y_base = y_cam + camera_y;
  z_base = -sin_pitch * x_cam + cos_pitch * z_cam + camera_z;

  if (!isfinite(x_base) || !isfinite(y_base) || !isfinite(z_base)) {
    return false;
  }
  if (x_base <= blind_spot) {
    return false;
  }

  return true;
}

__global__ void bevGroundMinKernel(
  const float * depth, const float * sky, int * bev_ground_min_z_mm, int width, int height,
  int downsample_factor, float fx, float fy, float cx, float cy, float camera_x, float camera_y,
  float camera_z, float cos_pitch, float sin_pitch, float blind_spot, float min_depth,
  float max_depth, float bev_resolution, int bev_origin_x, int bev_origin_y, int bev_width,
  int bev_height, float sky_threshold)
{
  const int out_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int out_y = blockIdx.y * blockDim.y + threadIdx.y;
  float x_base = 0.0f;
  float y_base = 0.0f;
  float z_base = 0.0f;
  float z_optical = 0.0f;
  if (!projectDepthToBase(
      depth, sky, width, height, downsample_factor, fx, fy, cx, cy, camera_x, camera_y,
      camera_z, cos_pitch, sin_pitch, blind_spot, min_depth, max_depth, sky_threshold, out_x,
      out_y, x_base, y_base, z_base, z_optical)) {
    return;
  }

  const int cell_x = static_cast<int>(floorf(x_base / bev_resolution));
  const int cell_y = static_cast<int>(floorf(y_base / bev_resolution));
  const int grid_x = cell_x - bev_origin_x;
  const int grid_y = cell_y - bev_origin_y;
  if (grid_x < 0 || grid_x >= bev_width || grid_y < 0 || grid_y >= bev_height) {
    return;
  }
  const int z_mm = __float2int_rn(z_base * 1000.0f);
  atomicMin(&bev_ground_min_z_mm[grid_y * bev_width + grid_x], z_mm);
}

__global__ void depthToFilteredCloudKernel(
  const float * depth, const float * sky, float * output_xyzi, unsigned int * output_count,
  const int * bev_ground_min_z_mm, int width, int height, int downsample_factor, float fx,
  float fy, float cx, float cy, float camera_x, float camera_y, float camera_z,
  float cos_pitch, float sin_pitch, float blind_spot, float min_z, float max_z,
  float min_depth, float max_depth, float bev_resolution, float bev_height_diff,
  int bev_origin_x, int bev_origin_y, int bev_width, int bev_height, float sky_threshold)
{
  const int out_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int out_y = blockIdx.y * blockDim.y + threadIdx.y;
  float x_base = 0.0f;
  float y_base = 0.0f;
  float z_base = 0.0f;
  float z_optical = 0.0f;
  if (!projectDepthToBase(
      depth, sky, width, height, downsample_factor, fx, fy, cx, cy, camera_x, camera_y,
      camera_z, cos_pitch, sin_pitch, blind_spot, min_depth, max_depth, sky_threshold, out_x,
      out_y, x_base, y_base, z_base, z_optical)) {
    return;
  }

  const int cell_x = static_cast<int>(floorf(x_base / bev_resolution));
  const int cell_y = static_cast<int>(floorf(y_base / bev_resolution));
  const int grid_x = cell_x - bev_origin_x;
  const int grid_y = cell_y - bev_origin_y;
  if (grid_x < 0 || grid_x >= bev_width || grid_y < 0 || grid_y >= bev_height) {
    return;
  }

  constexpr int kInvalidGroundMm = 0x7f7f7f7f;
  const int ground_mm = bev_ground_min_z_mm[grid_y * bev_width + grid_x];
  if (ground_mm == kInvalidGroundMm) {
    return;
  }
  const float height_above_ground = z_base - static_cast<float>(ground_mm) * 0.001f;
  const float min_height = fmaxf(min_z, bev_height_diff);
  if (height_above_ground < min_height || height_above_ground > max_z) {
    return;
  }

  const unsigned int output_index = atomicAdd(output_count, 1U);
  const unsigned int offset = output_index * 4U;
  output_xyzi[offset + 0U] = x_base;
  output_xyzi[offset + 1U] = y_base;
  output_xyzi[offset + 2U] = z_base;
  output_xyzi[offset + 3U] = z_optical;
}

}  // namespace

cudaError_t launchBgrToNchwPreprocess(
  const uint8_t * input_bgr, int src_width, int src_height, int src_step_bytes,
  float * output_nchw, int dst_width, int dst_height, cudaStream_t stream)
{
  const dim3 block(16, 16);
  const dim3 grid((dst_width + block.x - 1) / block.x, (dst_height + block.y - 1) / block.y);
  bgrToNchwPreprocessKernel<<<grid, block, 0, stream>>>(
    input_bgr, src_width, src_height, src_step_bytes, output_nchw, dst_width, dst_height);
  return cudaGetLastError();
}

cudaError_t launchDepthCleanScale(
  const float * depth, const float * sky, float * output_depth, int width, int height,
  float focal_scale, float sky_threshold, float sky_depth_cap, cudaStream_t stream)
{
  const dim3 block(16, 16);
  const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
  depthCleanScaleKernel<<<grid, block, 0, stream>>>(
    depth, sky, output_depth, width, height, focal_scale, sky_threshold, sky_depth_cap);
  return cudaGetLastError();
}

cudaError_t launchDepthResizeBilinear(
  const float * input_depth, int input_width, int input_height, float * output_depth,
  int output_width, int output_height, cudaStream_t stream)
{
  const dim3 block(16, 16);
  const dim3 grid((output_width + block.x - 1) / block.x, (output_height + block.y - 1) / block.y);
  depthResizeBilinearKernel<<<grid, block, 0, stream>>>(
    input_depth, input_width, input_height, output_depth, output_width, output_height);
  return cudaGetLastError();
}

cudaError_t launchDepthToFilteredCloud(
  const float * depth, const float * sky, float * output_xyzi, unsigned int * output_count,
  int * bev_ground_min_z_mm, int width, int height, int downsample_factor, float fx, float fy,
  float cx, float cy,
  float camera_x, float camera_y, float camera_z, float cos_pitch, float sin_pitch,
  float blind_spot, float min_z, float max_z, float min_depth, float max_depth,
  float bev_resolution, float bev_height_diff, int bev_origin_x, int bev_origin_y,
  int bev_width, int bev_height, float sky_threshold, cudaStream_t stream)
{
  const dim3 block(16, 16);
  const int sampled_width = (width + downsample_factor - 1) / downsample_factor;
  const int sampled_height = (height + downsample_factor - 1) / downsample_factor;
  const dim3 grid((sampled_width + block.x - 1) / block.x, (sampled_height + block.y - 1) / block.y);
  bevGroundMinKernel<<<grid, block, 0, stream>>>(
    depth, sky, bev_ground_min_z_mm, width, height, downsample_factor, fx, fy, cx, cy,
    camera_x, camera_y, camera_z, cos_pitch, sin_pitch, blind_spot, min_depth, max_depth,
    bev_resolution, bev_origin_x, bev_origin_y, bev_width, bev_height, sky_threshold);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    return err;
  }
  depthToFilteredCloudKernel<<<grid, block, 0, stream>>>(
    depth, sky, output_xyzi, output_count, bev_ground_min_z_mm, width, height, downsample_factor, fx, fy, cx, cy,
    camera_x, camera_y, camera_z, cos_pitch, sin_pitch, blind_spot, min_z, max_z, min_depth,
    max_depth, bev_resolution, bev_height_diff, bev_origin_x, bev_origin_y, bev_width,
    bev_height, sky_threshold);
  return cudaGetLastError();
}

}  // namespace depth_anything_v3
