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

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <cstring>
#include <iostream>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>

#include "depth_anything_v3/tensorrt_depth_anything.hpp"
#ifdef DEPTH_ANYTHING_V3_ENABLE_CUDA_KERNELS
#include "depth_anything_v3/cuda_kernels.hpp"
#endif
#include "cuda_utils/cuda_check_error.hpp"
#include "cuda_utils/cuda_unique_ptr.hpp"

namespace
{
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

static double elapsedMs(const Clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

static size_t volumeFromDims(const nvinfer1::Dims & dims, int batch_size)
{
  return std::accumulate(dims.d, dims.d + dims.nbDims, size_t{1},
    [batch_size](size_t acc, int dim) {
      return acc * (dim == -1 ? static_cast<size_t>(batch_size) : static_cast<size_t>(dim));
    });
}

// Simple depth to point cloud conversion using camera intrinsics
static void depthImageToPointCloud(
  const cv::Mat & depth_image,
  const sensor_msgs::msg::CameraInfo & camera_info,
  sensor_msgs::msg::PointCloud2 & cloud_msg,
  const std::string & frame_id,
  int downsample_factor = 1,
  const cv::Mat & rgb_image = cv::Mat(),
  const cv::Mat & non_sky_mask = cv::Mat(),
  depth_anything_v3::TensorRTDepthAnything::Profiling * profiling = nullptr)
{
  const auto setup_start = Clock::now();
  cloud_msg.header.frame_id = frame_id;
  
  const int downsampled_height = (depth_image.rows + downsample_factor - 1) / downsample_factor;
  const int downsampled_width = (depth_image.cols + downsample_factor - 1) / downsample_factor;
  
  cloud_msg.height = downsampled_height;
  cloud_msg.width = downsampled_width;
  cloud_msg.is_dense = false;
  cloud_msg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(cloud_msg);
  const bool has_color = !rgb_image.empty();
  if (has_color) {
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
  }

  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4]; 
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];

  if (profiling) {
    profiling->pointcloud_setup_ms = elapsedMs(setup_start);
  }

  const auto xyz_start = Clock::now();
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  const float bad_point = std::numeric_limits<float>::quiet_NaN();
  const bool use_mask = !non_sky_mask.empty();

  for (int v = 0; v < depth_image.rows; v += downsample_factor) {
    for (int u = 0; u < depth_image.cols; u += downsample_factor) {
      if (use_mask && non_sky_mask.at<uint8_t>(v, u) == 0) {
        *iter_x = *iter_y = *iter_z = bad_point;
        ++iter_x; ++iter_y; ++iter_z;
        continue;
      }

      const float depth = depth_image.at<float>(v, u);
      
      if (depth <= 0.0f || !std::isfinite(depth)) {
        *iter_x = *iter_y = *iter_z = bad_point;
      } else {
        *iter_x = static_cast<float>((u - cx) * depth / fx);
        *iter_y = static_cast<float>((v - cy) * depth / fy);
        *iter_z = depth;
      }
      ++iter_x; ++iter_y; ++iter_z;
    }
  }
  if (profiling) {
    profiling->pointcloud_xyz_fill_ms = elapsedMs(xyz_start);
  }

  if (has_color) {
    const auto rgb_start = Clock::now();
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
    for (int v = 0; v < depth_image.rows; v += downsample_factor) {
      for (int u = 0; u < depth_image.cols; u += downsample_factor) {
        if (use_mask && non_sky_mask.at<uint8_t>(v, u) == 0) {
          *iter_r = *iter_g = *iter_b = 0;
        } else {
          const cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(v, u);
          *iter_r = rgb[2];
          *iter_g = rgb[1];
          *iter_b = rgb[0];
        }
        ++iter_r; ++iter_g; ++iter_b;
      }
    }
    if (profiling) {
      profiling->pointcloud_rgb_fill_ms = elapsedMs(rgb_start);
    }
  }
}

} // anonymous namespace

namespace depth_anything_v3
{

TensorRTDepthAnything::TensorRTDepthAnything(
  const std::string & model_path, const std::string & precision,
  tensorrt_common::BuildConfig build_config, const bool use_gpu_preprocess,
  const bool use_gpu_postprocess, std::string /* calibration_image_list_path */,
  const tensorrt_common::BatchConfig & batch_config, const size_t max_workspace_size)
: batch_size_(batch_config[2]),
  use_gpu_preprocess_(use_gpu_preprocess),
  use_gpu_postprocess_(use_gpu_postprocess)
{
  src_width_ = -1;
  src_height_ = -1;

  if (!fs::exists(model_path)) {
    throw std::runtime_error("Model file does not exist: " + model_path);
  }

  // Initialize TensorRT common
  trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(
    model_path, precision, nullptr, batch_config, max_workspace_size, build_config);
  trt_common_->setup();

  auto * engine = trt_common_->getEngine();
  depth_elem_num_ = 0;
  sky_elem_num_ = 0;
  for (int i = 0; i < trt_common_->getNbIOTensors(); ++i) {
    const char * name = engine->getIOTensorName(i);
    const auto dims = trt_common_->getBindingDimensions(i);
    if (name && std::string(name) == "depth") {
      depth_elem_num_ = volumeFromDims(dims, batch_size_);
    } else if (name && std::string(name) == "sky") {
      sky_elem_num_ = volumeFromDims(dims, batch_size_);
    }
  }
  if (depth_elem_num_ == 0) {
    // Fallback to the first output binding if names are unavailable
    const auto dims = trt_common_->getBindingDimensions(1);
    depth_elem_num_ = volumeFromDims(dims, batch_size_);
  }
  if (sky_elem_num_ == 0) {
    throw std::runtime_error("Expected TensorRT engine to expose 'sky' output tensor, but none was found");
  }

  // Allocate GPU/CPU memory for outputs
  depth_d_ = cuda_utils::make_unique<float[]>(depth_elem_num_);
  depth_h_ = cuda_utils::make_unique_host<float[]>(depth_elem_num_, cudaHostAllocDefault);
  sky_d_ = cuda_utils::make_unique<float[]>(sky_elem_num_);
  sky_h_ = cuda_utils::make_unique_host<float[]>(sky_elem_num_, cudaHostAllocDefault);

  // Get input dimensions
  const auto input_dims = trt_common_->getBindingDimensions(0);
  const int input_channels = input_dims.d[1];
  input_height_ = input_dims.d[2]; 
  input_width_ = input_dims.d[3];
  
  // Allocate input memory
  const size_t input_elem_num = batch_size_ * input_channels * input_height_ * input_width_;
  input_d_ = cuda_utils::make_unique<float[]>(input_elem_num);
  input_h_.resize(input_elem_num);

}

void TensorRTDepthAnything::initPreprocessBuffer(int width, int height)
{
  src_width_ = width;
  src_height_ = height;
  scale_x_ = static_cast<double>(input_width_) / static_cast<double>(src_width_);
  scale_y_ = static_cast<double>(input_height_) / static_cast<double>(src_height_);
  
  const size_t image_size = static_cast<size_t>(src_width_) * src_height_ * 3;
  image_buf_d_ = cuda_utils::make_unique<unsigned char[]>(image_size * batch_size_);
}

bool TensorRTDepthAnything::doInference(
  const std::vector<cv::Mat> & images, 
  const sensor_msgs::msg::CameraInfo & camera_info,
  bool generate_pointcloud,
  int downsample_factor,
  bool colorize_pointcloud)
{
  profiling_ = Profiling{};
  if (images.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("TensorRTDepthAnything"), "No images provided for inference.");
    return false;
  }

  if (images.size() != static_cast<size_t>(batch_size_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TensorRTDepthAnything"),
      "Batch size mismatch. Expected: %d, got: %zu", batch_size_, images.size());
    return false;
  }

  const auto preprocess_start = Clock::now();
  preprocess(images);
  profiling_.preprocess_ms = elapsedMs(preprocess_start);

  // Run inference
  if (!infer()) {
    return false;
  }

  // Postprocess with downsampling
  cv::Mat rgb_for_pointcloud = generate_pointcloud && colorize_pointcloud ? images[0] : cv::Mat();
  postprocess(camera_info, generate_pointcloud, downsample_factor, rgb_for_pointcloud);
  
  return true;
}

void TensorRTDepthAnything::preprocess(const std::vector<cv::Mat> & images)
{
  const auto batch_size = images.size();
  auto input_dims = trt_common_->getBindingDimensions(0);
  if (input_dims.d[0] == -1) {
    input_dims.d[0] = batch_size_;
  }
  trt_common_->setBindingDimensions(0, input_dims);

  input_height_ = input_dims.d[2];
  input_width_ = input_dims.d[3];
  const int input_chan = input_dims.d[1];
  scale_x_ = static_cast<double>(input_width_) / static_cast<double>(src_width_);
  scale_y_ = static_cast<double>(input_height_) / static_cast<double>(src_height_);


  const size_t volume = batch_size * input_chan * input_height_ * input_width_;
#ifdef DEPTH_ANYTHING_V3_ENABLE_CUDA_KERNELS
  if (input_chan == 3 && use_gpu_preprocess_ && image_buf_d_) {
    const size_t image_bytes = static_cast<size_t>(src_width_) * src_height_ * 3;
    for (size_t n = 0; n < batch_size; ++n) {
      const auto & image = images[n];
      unsigned char * batch_image_d = image_buf_d_.get() + n * image_bytes;
      float * batch_input_d = input_d_.get() + n * input_chan * input_height_ * input_width_;
      CHECK_CUDA_ERROR(cudaMemcpy2DAsync(
        batch_image_d, static_cast<size_t>(src_width_) * 3, image.data, image.step,
        static_cast<size_t>(src_width_) * 3, src_height_, cudaMemcpyHostToDevice, *stream_));
      CHECK_CUDA_ERROR(launchBgrToNchwPreprocess(
        batch_image_d, src_width_, src_height_, src_width_ * 3, batch_input_d,
        input_width_, input_height_, *stream_));
    }
  } else {
#endif
    std::vector<cv::Mat> resized_images;
    resized_images.reserve(batch_size);
    for (const auto & image : images) {
      cv::Mat resized_image;
      cv::resize(image, resized_image, cv::Size(input_width_, input_height_), 0, 0, cv::INTER_CUBIC);
      resized_images.emplace_back(resized_image);
    }

    input_h_.assign(volume, 0.0f);
    const std::vector<float> mean{0.485f, 0.456f, 0.406f};
    const std::vector<float> std_vals{0.229f, 0.224f, 0.225f};

    const size_t strides_cv[4] = {
      static_cast<size_t>(input_width_ * input_chan * input_height_),
      static_cast<size_t>(input_width_ * input_chan),
      static_cast<size_t>(input_chan), 1};
    const size_t strides[4] = {
      static_cast<size_t>(input_height_ * input_width_ * input_chan),
      static_cast<size_t>(input_height_ * input_width_),
      static_cast<size_t>(input_width_), 1};

    for (size_t n = 0; n < batch_size; ++n) {
      const auto & img = resized_images[n];
      const auto * src_ptr = img.data;
      for (int h = 0; h < input_height_; ++h) {
        for (int w = 0; w < input_width_; ++w) {
          for (int c = 0; c < input_chan; ++c) {
            const size_t offset_cv =
              h * strides_cv[1] + w * strides_cv[2] + (input_chan - c - 1) * strides_cv[3];
            const size_t offset =
              n * strides[0] + c * strides[1] + h * strides[2] + w * strides[3];
            const float value = static_cast<float>(src_ptr[offset_cv]) / 255.0f;
            input_h_[offset] = (value - mean[c]) / std_vals[c];
          }
        }
      }
    }
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      input_d_.get(), input_h_.data(), input_h_.size() * sizeof(float), cudaMemcpyHostToDevice,
      *stream_));
#ifdef DEPTH_ANYTHING_V3_ENABLE_CUDA_KERNELS
  }
#endif

  auto * engine = trt_common_->getEngine();
  for (int i = 0; i < trt_common_->getNbIOTensors(); ++i) {
    const char * name = engine->getIOTensorName(i);
    const auto dims = trt_common_->getBindingDimensions(i);
    const size_t required_output_elems = volumeFromDims(dims, batch_size_);
    if (name && std::string(name) == "depth") {
      if (required_output_elems != depth_elem_num_) {
        depth_elem_num_ = required_output_elems;
        depth_d_ = cuda_utils::make_unique<float[]>(depth_elem_num_);
        depth_h_ = cuda_utils::make_unique_host<float[]>(depth_elem_num_, cudaHostAllocDefault);
      }
    } else if (name && std::string(name) == "sky") {
      if (required_output_elems != sky_elem_num_) {
        sky_elem_num_ = required_output_elems;
        sky_d_ = cuda_utils::make_unique<float[]>(sky_elem_num_);
        sky_h_ = cuda_utils::make_unique_host<float[]>(sky_elem_num_, cudaHostAllocDefault);
      }
    }
  }

}

bool TensorRTDepthAnything::infer()
{
  auto * context = trt_common_->getContext();
  auto * engine = trt_common_->getEngine();

  extra_output_buffers_.clear();

  for (int i = 0; i < trt_common_->getNbIOTensors(); ++i) {
    const char * name = engine->getIOTensorName(i);
    const std::string tensor_name = name ? std::string(name) : std::string();
    const bool is_input = engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;

    void * buffer_ptr = nullptr;
    if (is_input || tensor_name.find("input") != std::string::npos ||
        tensor_name.find("image") != std::string::npos) {
      buffer_ptr = input_d_.get();
    } else if (tensor_name == "depth" || tensor_name.find("depth") != std::string::npos) {
      buffer_ptr = depth_d_.get();
    } else if (tensor_name == "sky" || tensor_name.find("sky") != std::string::npos) {
      if (!sky_d_ && sky_elem_num_ > 0) {
        sky_d_ = cuda_utils::make_unique<float[]>(sky_elem_num_);
        sky_h_ = cuda_utils::make_unique_host<float[]>(sky_elem_num_, cudaHostAllocDefault);
      }
      buffer_ptr = sky_d_ ? static_cast<void *>(sky_d_.get()) : static_cast<void *>(depth_d_.get());
    } else {
      const auto dims = trt_common_->getBindingDimensions(i);
      const size_t elem_count = volumeFromDims(dims, batch_size_);
      extra_output_buffers_.push_back(cuda_utils::make_unique<float[]>(elem_count));
      buffer_ptr = extra_output_buffers_.back().get();
    }

    context->setTensorAddress(name, buffer_ptr);
  }

  const auto inference_start = Clock::now();
  if (!trt_common_->enqueueV3(*stream_)) {
    return false;
  }
  if (profiling_enabled_) {
    CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
  }
  profiling_.tensorrt_inference_ms = elapsedMs(inference_start);

  if (!use_gpu_postprocess_) {
    const auto output_copy_start = Clock::now();
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      depth_h_.get(), depth_d_.get(), depth_elem_num_ * sizeof(float),
      cudaMemcpyDeviceToHost, *stream_));

    if (sky_d_ && sky_h_) {
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        sky_h_.get(), sky_d_.get(), sky_elem_num_ * sizeof(float),
        cudaMemcpyDeviceToHost, *stream_));
    }

    CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
    profiling_.depth_output_copy_ms = elapsedMs(output_copy_start);
  }

  return true;
}

void TensorRTDepthAnything::postprocess(
  const sensor_msgs::msg::CameraInfo & camera_info, bool generate_pointcloud,
  int downsample_factor, const cv::Mat & rgb_image)
{
  const auto postprocess_start = Clock::now();
  const auto output_dims = trt_common_->getBindingDimensions(1);
  const int height = output_dims.nbDims > 2 ? output_dims.d[2] : input_height_;
  const int width = output_dims.nbDims > 3 ? output_dims.d[3] : input_width_;
  model_depth_width_ = width;
  model_depth_height_ = height;
  const size_t plane_size = static_cast<size_t>(height) * width;

  // Use original intrinsics for metric conversion per spec.
  const double fx = camera_info.k[0] * scale_x_;
  const double fy = camera_info.k[4] * scale_y_;
  const double focal_pixels = 0.5 * (fx + fy);
  const double focal_scale = focal_pixels > 0.0 ? focal_pixels / 300.0 : 1.0;

#ifdef DEPTH_ANYTHING_V3_ENABLE_CUDA_KERNELS
  if (use_gpu_postprocess_) {
    if (model_depth_elem_num_ != plane_size) {
      model_depth_elem_num_ = plane_size;
      model_depth_d_ = cuda_utils::make_unique<float[]>(model_depth_elem_num_);
    }
    const size_t resized_plane_size = static_cast<size_t>(src_width_) * src_height_;
    if (resized_depth_elem_num_ != resized_plane_size) {
      resized_depth_elem_num_ = resized_plane_size;
      resized_depth_d_ = cuda_utils::make_unique<float[]>(resized_depth_elem_num_);
    }

    CHECK_CUDA_ERROR(launchDepthCleanScale(
      depth_d_.get(), sky_d_.get(), model_depth_d_.get(), width, height,
      static_cast<float>(focal_scale), sky_threshold_, sky_depth_cap_, *stream_));
    if (profiling_enabled_) {
      CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
    }
    profiling_.depth_postprocess_ms = elapsedMs(postprocess_start);

    const auto resize_start = Clock::now();
    CHECK_CUDA_ERROR(launchDepthResizeBilinear(
      model_depth_d_.get(), width, height, resized_depth_d_.get(), src_width_, src_height_,
      *stream_));
    if (profiling_enabled_) {
      CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
    }
    profiling_.depth_resize_ms = elapsedMs(resize_start);

    const auto output_copy_start = Clock::now();
    depth_image_.create(src_height_, src_width_, CV_32FC1);
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      depth_image_.data, resized_depth_d_.get(), resized_depth_elem_num_ * sizeof(float),
      cudaMemcpyDeviceToHost, *stream_));
    if (generate_pointcloud) {
      model_depth_.create(height, width, CV_32FC1);
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        model_depth_.data, model_depth_d_.get(), model_depth_elem_num_ * sizeof(float),
        cudaMemcpyDeviceToHost, *stream_));
      if (sky_d_ && sky_h_) {
        CHECK_CUDA_ERROR(cudaMemcpyAsync(
          sky_h_.get(), sky_d_.get(), sky_elem_num_ * sizeof(float),
          cudaMemcpyDeviceToHost, *stream_));
      }
    }
    CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
    profiling_.depth_output_copy_ms = elapsedMs(output_copy_start);

    sky_mask_.release();
    if (generate_pointcloud && sky_h_) {
      cv::Mat sky_pred(height, width, CV_32FC1, const_cast<float *>(sky_h_.get()));
      sky_mask_ = sky_pred < sky_threshold_;
    }

    cv::Mat colorized = rgb_image;
    if (!colorized.empty() &&
        (colorized.rows != depth_image_.rows || colorized.cols != depth_image_.cols)) {
      cv::resize(colorized, colorized, depth_image_.size(), 0, 0, cv::INTER_LINEAR);
    }

    if (generate_pointcloud) {
      const auto pointcloud_start = Clock::now();
      buildPointCloud(camera_info, downsample_factor, colorized);
      profiling_.depth_to_pointcloud_total_ms = elapsedMs(pointcloud_start);
    }
    if (ground_filter_params_.enabled) {
      buildGpuFilteredPointCloud(camera_info);
    }
    return;
  }
#endif

  // Use depth output directly
  const float * depth_ptr = depth_h_.get();
  model_depth_.create(height, width, CV_32FC1);
  std::memcpy(model_depth_.data, depth_ptr, plane_size * sizeof(float));

  // Sky output
  sky_mask_.release();
  cv::Mat sky_pred(height, width, CV_32FC1, const_cast<float *>(sky_h_.get()));
  sky_mask_ = sky_pred < sky_threshold_;

  // Inspect raw output before scaling.
  double raw_min = 0.0, raw_max = 0.0;
  cv::minMaxLoc(model_depth_, &raw_min, &raw_max);
  RCLCPP_DEBUG(
    rclcpp::get_logger("TensorRTDepthAnything"),
    "Raw net output min/max: %.6f / %.6f", raw_min, raw_max);

  // Clean and scale to metric depth.
  cv::Mat depth_map = model_depth_.clone();
  depth_map.setTo(0.0f, depth_map <= 0.0f);

  depth_map *= static_cast<float>(focal_scale);

  // Handle sky: set sky pixels to max depth derived from non-sky regions.
  if (!sky_mask_.empty()) {
    std::vector<float> valid_depths;
    valid_depths.reserve(plane_size);
    const uint8_t * mask_ptr = sky_mask_.ptr<uint8_t>(0);
    const float * depth_ptr_flat = reinterpret_cast<const float *>(depth_map.data);
    for (size_t idx = 0; idx < plane_size; ++idx) {
      if (mask_ptr[idx]) {
        const float val = depth_ptr_flat[idx];
        if (std::isfinite(val) && val > 0.0f) {
          valid_depths.push_back(val);
        }
      }
    }
    if (!valid_depths.empty()) {
      size_t sample_size = valid_depths.size();
      const size_t max_sample = 100000;
      if (sample_size > max_sample) {
        const size_t step = sample_size / max_sample;
        std::vector<float> sampled;
        sampled.reserve(max_sample);
        for (size_t i = 0; i < sample_size && sampled.size() < max_sample; i += step) {
          sampled.push_back(valid_depths[i]);
        }
        valid_depths.swap(sampled);
      }
      const size_t idx = static_cast<size_t>(0.99 * (valid_depths.size() - 1));
      std::nth_element(valid_depths.begin(), valid_depths.begin() + idx, valid_depths.end());
      const float max_depth = std::min(valid_depths[idx], sky_depth_cap_);
      cv::Mat depth_map_reshaped(height, width, CV_32FC1, depth_map.data);
      depth_map_reshaped.setTo(max_depth, ~sky_mask_);
    }
  }

  // Persist scaled depth for point cloud generation at network resolution.
  model_depth_ = depth_map.clone();

  double min_metric = 0.0, max_metric = 0.0;
  cv::minMaxLoc(model_depth_, &min_metric, &max_metric);
  RCLCPP_DEBUG(
    rclcpp::get_logger("TensorRTDepthAnything"),
    "Metric depth (model) min/max: %.6f / %.6f meters", min_metric, max_metric);
  RCLCPP_DEBUG(
    rclcpp::get_logger("TensorRTDepthAnything"),
    "Focal length in pixels: %.6f (scale factor: %.6f)", focal_pixels, focal_scale);
  profiling_.depth_postprocess_ms = elapsedMs(postprocess_start);

  const auto resize_start = Clock::now();
  cv::resize(model_depth_, depth_image_, cv::Size(src_width_, src_height_), 0, 0, cv::INTER_CUBIC);
  profiling_.depth_resize_ms = elapsedMs(resize_start);

  cv::Mat colorized = rgb_image;
  if (!colorized.empty() &&
      (colorized.rows != depth_image_.rows || colorized.cols != depth_image_.cols)) {
    cv::resize(colorized, colorized, depth_image_.size(), 0, 0, cv::INTER_LINEAR);
  }

  if (generate_pointcloud) {
    const auto pointcloud_start = Clock::now();
    buildPointCloud(camera_info, downsample_factor, colorized);
    profiling_.depth_to_pointcloud_total_ms = elapsedMs(pointcloud_start);
  }
  if (ground_filter_params_.enabled) {
    buildGpuFilteredPointCloud(camera_info);
  }
}


void TensorRTDepthAnything::buildPointCloud(
  const sensor_msgs::msg::CameraInfo & camera_info, int downsample_factor,
  const cv::Mat & rgb_image)
{

  // Resize color to match model output if provided.
  cv::Mat color;
  if (!rgb_image.empty()) {
    if (rgb_image.type() == CV_8UC3) {
      color = rgb_image.clone();
    } else {
      rgb_image.convertTo(color, CV_8UC3);
    }
    if (color.rows != model_depth_.rows || color.cols != model_depth_.cols) {
      cv::resize(color, color, model_depth_.size(), 0, 0, cv::INTER_LINEAR);
    }
  }

  // Scale intrinsics to model output resolution.
  sensor_msgs::msg::CameraInfo cam_scaled = camera_info;
  cam_scaled.k[0] = camera_info.k[0] * scale_x_;
  cam_scaled.k[4] = camera_info.k[4] * scale_y_;
  cam_scaled.k[2] = camera_info.k[2] * scale_x_;
  cam_scaled.k[5] = camera_info.k[5] * scale_y_;
  cam_scaled.width = model_depth_.cols;
  cam_scaled.height = model_depth_.rows;

  const std::string frame_id =
    camera_info.header.frame_id.empty() ? "camera_link" : camera_info.header.frame_id;

  depthImageToPointCloud(
    model_depth_, cam_scaled, point_cloud_, frame_id, downsample_factor, color, sky_mask_,
    &profiling_);

  // Preserve original timestamp
  point_cloud_.header.stamp = camera_info.header.stamp;
}

void TensorRTDepthAnything::packFilteredPointCloud(
  const float * xyzi, unsigned int count, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  filtered_point_cloud_.header.stamp = stamp;
  filtered_point_cloud_.header.frame_id = frame_id;
  filtered_point_cloud_.height = 1;
  filtered_point_cloud_.width = count;
  filtered_point_cloud_.is_dense = true;
  filtered_point_cloud_.is_bigendian = false;
  filtered_point_cloud_.fields = {
    sensor_msgs::msg::PointField()
  };
  filtered_point_cloud_.fields.resize(4);
  const char * names[4] = {"x", "y", "z", "intensity"};
  for (size_t i = 0; i < 4; ++i) {
    filtered_point_cloud_.fields[i].name = names[i];
    filtered_point_cloud_.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
    filtered_point_cloud_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    filtered_point_cloud_.fields[i].count = 1;
  }
  filtered_point_cloud_.point_step = 4 * sizeof(float);
  filtered_point_cloud_.row_step = filtered_point_cloud_.point_step * count;
  filtered_point_cloud_.data.resize(static_cast<size_t>(filtered_point_cloud_.row_step));
  if (count > 0) {
    std::memcpy(filtered_point_cloud_.data.data(), xyzi, filtered_point_cloud_.data.size());
  }
}

void TensorRTDepthAnything::buildGpuFilteredPointCloud(
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  const auto filter_start = Clock::now();
  const auto & p = ground_filter_params_;
  const int downsample = std::max(1, p.downsample_factor);
  const std::string frame_id = "base_link";
  const float bev_resolution = std::max(0.01f, p.bev_resolution);
  const float reach = std::max(p.max_depth, p.blind_spot + 1.0f);
  const float x_min = p.blind_spot;
  const float x_max = reach + std::abs(p.camera_x) + std::abs(p.camera_z) + 1.0f;
  const float y_extent = reach + std::abs(p.camera_y) + 1.0f;
  const int bev_origin_x = static_cast<int>(std::floor(x_min / bev_resolution));
  const int bev_max_x = static_cast<int>(std::floor(x_max / bev_resolution));
  const int bev_origin_y = static_cast<int>(std::floor(-y_extent / bev_resolution));
  const int bev_max_y = static_cast<int>(std::floor(y_extent / bev_resolution));
  const int bev_width = std::max(1, bev_max_x - bev_origin_x + 1);
  const int bev_height = std::max(1, bev_max_y - bev_origin_y + 1);
  const size_t bev_cell_count = static_cast<size_t>(bev_width) * static_cast<size_t>(bev_height);

  const double fx_d = camera_info.k[0] * scale_x_;
  const double fy_d = camera_info.k[4] * scale_y_;
  const double cx_d = camera_info.k[2] * scale_x_;
  const double cy_d = camera_info.k[5] * scale_y_;
  if (fx_d <= 0.0 || fy_d <= 0.0 || !std::isfinite(fx_d) || !std::isfinite(fy_d)) {
    packFilteredPointCloud(nullptr, 0U, frame_id, camera_info.header.stamp);
    return;
  }

#ifdef DEPTH_ANYTHING_V3_ENABLE_CUDA_KERNELS
  if (use_gpu_postprocess_ && model_depth_d_) {
    const int width = model_depth_width_ > 0 ? model_depth_width_ : input_width_;
    const int height = model_depth_height_ > 0 ? model_depth_height_ : input_height_;
    const size_t sampled_width = static_cast<size_t>((width + downsample - 1) / downsample);
    const size_t sampled_height = static_cast<size_t>((height + downsample - 1) / downsample);
    const size_t capacity = sampled_width * sampled_height;
    if (filtered_cloud_capacity_ != capacity) {
      filtered_cloud_capacity_ = capacity;
      filtered_cloud_d_ = cuda_utils::make_unique<float[]>(filtered_cloud_capacity_ * 4);
      filtered_cloud_h_ = cuda_utils::make_unique_host<float[]>(
        filtered_cloud_capacity_ * 4, cudaHostAllocDefault);
      filtered_count_d_ = cuda_utils::make_unique<unsigned int[]>(1);
    }
    if (bev_ground_cell_count_ != bev_cell_count) {
      bev_ground_cell_count_ = bev_cell_count;
      bev_ground_min_z_d_ = cuda_utils::make_unique<int[]>(bev_ground_cell_count_);
    }

    CHECK_CUDA_ERROR(cudaMemsetAsync(filtered_count_d_.get(), 0, sizeof(unsigned int), *stream_));
    CHECK_CUDA_ERROR(cudaMemsetAsync(
      bev_ground_min_z_d_.get(), 0x7f, bev_ground_cell_count_ * sizeof(int), *stream_));
    CHECK_CUDA_ERROR(launchDepthToFilteredCloud(
      model_depth_d_.get(), sky_d_.get(), filtered_cloud_d_.get(), filtered_count_d_.get(),
      bev_ground_min_z_d_.get(), width, height, downsample, static_cast<float>(fx_d), static_cast<float>(fy_d),
      static_cast<float>(cx_d), static_cast<float>(cy_d), p.camera_x, p.camera_y, p.camera_z,
      std::cos(p.camera_pitch), std::sin(p.camera_pitch), p.blind_spot, p.min_z, p.max_z,
      p.min_depth, p.max_depth, bev_resolution, p.bev_height_diff, bev_origin_x, bev_origin_y,
      bev_width, bev_height, sky_threshold_, *stream_));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
    profiling_.gpu_ground_filter_ms = elapsedMs(filter_start);

    const auto copy_start = Clock::now();
    unsigned int count = 0;
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      &count, filtered_count_d_.get(), sizeof(unsigned int), cudaMemcpyDeviceToHost, *stream_));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
    count = std::min<unsigned int>(count, static_cast<unsigned int>(filtered_cloud_capacity_));
    if (count > 0) {
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        filtered_cloud_h_.get(), filtered_cloud_d_.get(), static_cast<size_t>(count) * 4 * sizeof(float),
        cudaMemcpyDeviceToHost, *stream_));
      CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
    }
    profiling_.filtered_pointcloud_copy_ms = elapsedMs(copy_start);
    packFilteredPointCloud(filtered_cloud_h_.get(), count, frame_id, camera_info.header.stamp);
    return;
  }
#endif

  std::vector<float> xyzi;
  const int width = model_depth_.cols;
  const int height = model_depth_.rows;
  struct CandidatePoint
  {
    float x;
    float y;
    float z;
    float depth;
    size_t cell;
  };
  std::vector<CandidatePoint> candidates;
  candidates.reserve(static_cast<size_t>((width + downsample - 1) / downsample) *
                     static_cast<size_t>((height + downsample - 1) / downsample));
  constexpr int invalid_ground_mm = 0x7f7f7f7f;
  std::vector<int> local_ground_mm(bev_cell_count, invalid_ground_mm);
  const float fx = static_cast<float>(fx_d);
  const float fy = static_cast<float>(fy_d);
  const float cx = static_cast<float>(cx_d);
  const float cy = static_cast<float>(cy_d);
  const float cos_pitch = std::cos(p.camera_pitch);
  const float sin_pitch = std::sin(p.camera_pitch);
  for (int v = 0; v < height; v += downsample) {
    for (int u = 0; u < width; u += downsample) {
      if (!sky_mask_.empty() && sky_mask_.at<uint8_t>(v, u) == 0) {
        continue;
      }
      const float z_optical = model_depth_.at<float>(v, u);
      if (!std::isfinite(z_optical) || z_optical < p.min_depth || z_optical > p.max_depth) {
        continue;
      }
      const float x_optical = (static_cast<float>(u) - cx) * z_optical / fx;
      const float y_optical = (static_cast<float>(v) - cy) * z_optical / fy;
      const float x_cam = z_optical;
      const float y_cam = -x_optical;
      const float z_cam = -y_optical;
      const float x_base = cos_pitch * x_cam + sin_pitch * z_cam + p.camera_x;
      const float y_base = y_cam + p.camera_y;
      const float z_base = -sin_pitch * x_cam + cos_pitch * z_cam + p.camera_z;
      if (!std::isfinite(x_base) || !std::isfinite(y_base) || !std::isfinite(z_base) ||
          x_base <= p.blind_spot) {
        continue;
      }
      const int cell_x = static_cast<int>(std::floor(x_base / bev_resolution)) - bev_origin_x;
      const int cell_y = static_cast<int>(std::floor(y_base / bev_resolution)) - bev_origin_y;
      if (cell_x < 0 || cell_x >= bev_width || cell_y < 0 || cell_y >= bev_height) {
        continue;
      }
      const size_t cell = static_cast<size_t>(cell_y) * static_cast<size_t>(bev_width) +
                          static_cast<size_t>(cell_x);
      const int z_mm = static_cast<int>(std::lround(z_base * 1000.0f));
      local_ground_mm[cell] = std::min(local_ground_mm[cell], z_mm);
      candidates.push_back({x_base, y_base, z_base, z_optical, cell});
    }
  }
  const float min_height = std::max(p.min_z, p.bev_height_diff);
  xyzi.reserve(candidates.size() * 4);
  for (const auto & point : candidates) {
    const int ground_mm = local_ground_mm[point.cell];
    if (ground_mm == invalid_ground_mm) {
      continue;
    }
    const float height_above_ground = point.z - static_cast<float>(ground_mm) * 0.001f;
    if (height_above_ground < min_height || height_above_ground > p.max_z) {
      continue;
    }
    xyzi.insert(xyzi.end(), {point.x, point.y, point.z, point.depth});
  }
  profiling_.gpu_ground_filter_ms = elapsedMs(filter_start);
  packFilteredPointCloud(
    xyzi.empty() ? nullptr : xyzi.data(), static_cast<unsigned int>(xyzi.size() / 4), frame_id,
    camera_info.header.stamp);
}

const cv::Mat& TensorRTDepthAnything::getDepthImage() const
{
  return depth_image_;
}

const sensor_msgs::msg::PointCloud2& TensorRTDepthAnything::getPointCloud() const
{
  return point_cloud_;
}

const sensor_msgs::msg::PointCloud2& TensorRTDepthAnything::getFilteredPointCloud() const
{
  return filtered_point_cloud_;
}

void TensorRTDepthAnything::printProfiling()
{
  trt_common_->printProfiling();
}

} // namespace depth_anything_v3
