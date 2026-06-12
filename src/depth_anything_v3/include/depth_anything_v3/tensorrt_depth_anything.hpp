// Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEPTH_ANYTHING_V3__TENSORRT_DEPTH_ANYTHING_HPP_
#define DEPTH_ANYTHING_V3__TENSORRT_DEPTH_ANYTHING_HPP_

#include <cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_utils/stream_unique_ptr.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <tensorrt_common/tensorrt_common.hpp>
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace depth_anything_v3
{
using cuda_utils::CudaUniquePtr;
using cuda_utils::CudaUniquePtrHost;
using cuda_utils::makeCudaStream;
using cuda_utils::StreamUniquePtr;

/**
 * @class TensorRTDepthAnything
 * @brief TensorRT DepthAnythingV3 wrapper for depth estimation and point cloud generation
 */
class TensorRTDepthAnything
{
public:
  struct Profiling
  {
    double preprocess_ms{};
    double tensorrt_inference_ms{};
    double depth_output_copy_ms{};
    double depth_postprocess_ms{};
    double depth_resize_ms{};
    double pointcloud_setup_ms{};
    double pointcloud_xyz_fill_ms{};
    double pointcloud_rgb_fill_ms{};
    double depth_to_pointcloud_total_ms{};
    double gpu_ground_filter_ms{};
    double filtered_pointcloud_copy_ms{};
  };

  struct GroundFilterParams
  {
    bool enabled{false};
    int downsample_factor{4};
    float blind_spot{0.3f};
    float min_z{0.08f};
    float max_z{1.5f};
    float min_depth{0.2f};
    float max_depth{12.0f};
    float bev_resolution{0.15f};
    float bev_height_diff{0.12f};
    float camera_x{0.15f};
    float camera_y{0.0f};
    float camera_z{1.0f};
    float camera_pitch{0.0f};
  };

  /**
   * @brief Construct TensorRTDepthAnything.
   * @param[in] model_path ONNX model_path
   * @param[in] precision precision for inference
   * @param[in] build_config configuration including precision, calibration method, etc.
   * @param[in] use_gpu_preprocess whether use cuda gpu for preprocessing
   * @param[in] calibration_image_list_file path for calibration files
   * @param[in] batch_config configuration for batched execution
   * @param[in] max_workspace_size maximum workspace for building TensorRT engine
   */
  TensorRTDepthAnything(
    const std::string & model_path, const std::string & precision,
    const tensorrt_common::BuildConfig build_config = tensorrt_common::BuildConfig(),
    const bool use_gpu_preprocess = false, const bool use_gpu_postprocess = false,
    std::string calibration_image_list_file = std::string(), const tensorrt_common::BatchConfig & batch_config = {1, 1, 1},
    const size_t max_workspace_size = (1 << 30));

  /**
   * @brief run inference including pre-process and post-process
   * @param[in] images batched images
   * @param[in] camera_info camera calibration info for point cloud generation
   * @param[in] downsample_factor only publish every Nth point (1 = no downsampling)
   * @param[in] colorize_pointcloud whether to colorize point cloud with RGB
   */
  bool doInference(
    const std::vector<cv::Mat> & images, const sensor_msgs::msg::CameraInfo & camera_info,
    bool generate_pointcloud = true, int downsample_factor = 1, bool colorize_pointcloud = false);

  void initPreprocessBuffer(int width, int height);

  /**
   * @brief output TensorRT profiles for each layer
   */
  void printProfiling(void);

  /**
   * @brief Get the depth image result
   * @return depth image as cv::Mat (const reference)
   */
  const cv::Mat& getDepthImage() const;

  /**
   * @brief Get the point cloud result
   * @return point cloud as ROS2 PointCloud2 message (const reference)
   */
  const sensor_msgs::msg::PointCloud2& getPointCloud() const;
  const sensor_msgs::msg::PointCloud2& getFilteredPointCloud() const;

  const Profiling & getProfiling() const { return profiling_; }
  void setProfilingEnabled(bool enabled) { profiling_enabled_ = enabled; }
  void setGroundFilterParams(const GroundFilterParams & params) { ground_filter_params_ = params; }

private:
  /**
   * @brief run preprocess including resizing, letterbox, NHWC2NCHW and toFloat on CPU
   * @param[in] images batching images
   */
  void preprocess(const std::vector<cv::Mat> & images);

  /**
   * @brief perform TensorRT inference
   */
  bool infer();

  /**
   * @brief postprocess inference results to generate depth and point cloud
   * @param[in] camera_info camera calibration for point cloud generation
   * @param[in] downsample_factor downsampling factor for point cloud
   * @param[in] rgb_image optional RGB image for colorizing point cloud
   */
  void postprocess(
    const sensor_msgs::msg::CameraInfo & camera_info, bool generate_pointcloud = true,
    int downsample_factor = 1, const cv::Mat & rgb_image = cv::Mat());

  /**
   * @brief Build point cloud from depth image using camera intrinsics
   * @param[in] camera_info camera calibration parameters
   * @param[in] downsample_factor only publish every Nth point
   * @param[in] rgb_image optional RGB image for colorizing point cloud
   */
  void buildPointCloud(
    const sensor_msgs::msg::CameraInfo & camera_info, int downsample_factor,
    const cv::Mat & rgb_image);
  void buildGpuFilteredPointCloud(const sensor_msgs::msg::CameraInfo & camera_info);
  void packFilteredPointCloud(
    const float * xyzi, unsigned int count, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp);
public:
  void setSkyThreshold(float threshold) { sky_threshold_ = threshold; }
  void setSkyDepthCap(float depth_cap) { sky_depth_cap_ = depth_cap; }

  std::unique_ptr<tensorrt_common::TrtCommon> trt_common_;

  // Input/output buffers
  std::vector<float> input_h_;
  CudaUniquePtr<float[]> input_d_;

  // Output buffer for predicted depth
  CudaUniquePtr<float[]> depth_d_;
  CudaUniquePtrHost<float[]> depth_h_;
  size_t depth_elem_num_{};
  // Output buffer for predicted sky
  CudaUniquePtr<float[]> sky_d_;
  CudaUniquePtrHost<float[]> sky_h_;
  size_t sky_elem_num_{};
  std::vector<CudaUniquePtr<float[]>> extra_output_buffers_;
  cv::Mat model_depth_;
  cv::Mat sky_mask_;

  StreamUniquePtr stream_{makeCudaStream()};

  int batch_size_;

  // preprocessing parameters
  bool use_gpu_preprocess_;
  bool use_gpu_postprocess_;
  CudaUniquePtrHost<unsigned char[]> image_buf_h_;
  CudaUniquePtr<unsigned char[]> image_buf_d_;
  CudaUniquePtr<float[]> model_depth_d_;
  CudaUniquePtr<float[]> resized_depth_d_;
  CudaUniquePtr<float[]> filtered_cloud_d_;
  CudaUniquePtrHost<float[]> filtered_cloud_h_;
  CudaUniquePtr<unsigned int[]> filtered_count_d_;
  CudaUniquePtr<int[]> bev_ground_min_z_d_;
  size_t model_depth_elem_num_{};
  size_t resized_depth_elem_num_{};
  size_t filtered_cloud_capacity_{};
  size_t bev_ground_cell_count_{};
  int model_depth_width_{};
  int model_depth_height_{};

  int src_width_;
  int src_height_;
  int input_width_ = 504;
  int input_height_ = 280;

  // Results
  cv::Mat depth_image_;
  double scale_x_{1.0};
  double scale_y_{1.0};
  float sky_threshold_{0.3f};
  float sky_depth_cap_{200.0f};
  sensor_msgs::msg::PointCloud2 point_cloud_;
  sensor_msgs::msg::PointCloud2 filtered_point_cloud_;
  GroundFilterParams ground_filter_params_{};
  Profiling profiling_{};
  bool profiling_enabled_{false};
};

}  // namespace depth_anything_v3

#endif  // DEPTH_ANYTHING_V3__TENSORRT_DEPTH_ANYTHING_HPP_
