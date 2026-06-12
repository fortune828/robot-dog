#!/usr/bin/env bash
set -euo pipefail

ROBOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONDA_PREFIX="${ROBOTDOG_CONDA_PREFIX:-/home/ubuntu/bl/miniconda3/envs/robotdog}"
MODEL_DIR="$ROBOT/models/da3"
TRT_HEADERS="$ROBOT/third_party/tensorrt/include"
TRT_PREFIX="$ROBOT/.local/tensorrt"
CUDA_PREFIX="$ROBOT/.local/cuda"
TRT_LIBS="$CONDA_PREFIX/lib/python3.10/site-packages/tensorrt_libs"
MODEL="$MODEL_DIR/DA3METRIC-LARGE.onnx"
ENGINE="$MODEL_DIR/DA3METRIC-LARGE.fp16-batch1.engine"

[[ -f "$MODEL" ]] || { echo "Missing model: $MODEL"; exit 1; }
[[ -f "$TRT_HEADERS/NvInfer.h" ]] || { echo "Missing TensorRT headers: $TRT_HEADERS"; exit 1; }
[[ -x "$CONDA_PREFIX/bin/nvcc" ]] || { echo "Missing nvcc in robotdog: $CONDA_PREFIX/bin/nvcc"; exit 1; }
[[ -d "$TRT_LIBS" ]] || { echo "Missing TensorRT libs in robotdog: $TRT_LIBS"; exit 1; }

mkdir -p "$TRT_PREFIX/lib" "$CUDA_PREFIX/bin" "$CUDA_PREFIX/targets"
ln -sfn "$CONDA_PREFIX/bin/nvcc" "$CUDA_PREFIX/bin/nvcc"
ln -sfn "$CONDA_PREFIX/bin/crt" "$CUDA_PREFIX/bin/crt"
ln -sfn "$CONDA_PREFIX/targets/x86_64-linux/include" "$CUDA_PREFIX/include"
ln -sfn "$CONDA_PREFIX/targets/x86_64-linux" "$CUDA_PREFIX/targets/x86_64-linux"
ln -sfn "$CONDA_PREFIX/targets/x86_64-linux/lib" "$CUDA_PREFIX/lib64"
ln -sfn "$CONDA_PREFIX/nvvm" "$CUDA_PREFIX/nvvm"

for lib in libnvinfer.so.10 libnvinfer_plugin.so.10 libnvonnxparser.so.10; do
  [[ -f "$TRT_LIBS/$lib" ]] || { echo "Missing TensorRT library: $TRT_LIBS/$lib"; exit 1; }
  ln -sfn "$TRT_LIBS/$lib" "$TRT_PREFIX/lib/$lib"
  ln -sfn "$lib" "$TRT_PREFIX/lib/${lib%.10}"
done

set +u
source /opt/ros/humble/setup.bash
set -u

export CC=/usr/bin/gcc
export CXX=/usr/bin/g++
export CUDA_HOME="$CUDA_PREFIX"
export CUDA_PATH="$CUDA_PREFIX"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-1}"
export NVCC_PREPEND_FLAGS="-I$CONDA_PREFIX/targets/x86_64-linux/include -L$CONDA_PREFIX/targets/x86_64-linux/lib -L$CONDA_PREFIX/lib"
export PATH="$CONDA_PREFIX/nvvm/bin:$CONDA_PREFIX/bin:$PATH"
export LD_LIBRARY_PATH="$TRT_PREFIX/lib:$CONDA_PREFIX/lib:$CONDA_PREFIX/targets/x86_64-linux/lib:$TRT_LIBS:${LD_LIBRARY_PATH:-}"

cd "$ROBOT"
colcon build --symlink-install --packages-select depth_anything_v3 --cmake-args \
  -DBUILD_TESTING=OFF \
  -DDEPTH_ANYTHING_V3_ENABLE_CUDA_KERNELS=ON \
  -DCMAKE_CUDA_ARCHITECTURES=89 \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++ \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX;$CONDA_PREFIX/targets/x86_64-linux;/opt/ros/humble;/usr;/usr/lib/x86_64-linux-gnu/cmake" \
  -DCUDA_TOOLKIT_ROOT_DIR="$CUDA_PREFIX" \
  -DCMAKE_CUDA_COMPILER="$CONDA_PREFIX/bin/nvcc" \
  -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog \
  -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt \
  -Dconsole_bridge_DIR=/usr/lib/x86_64-linux-gnu/console_bridge/cmake \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DTINYXML2_LIBRARY=/usr/lib/x86_64-linux-gnu/libtinyxml2.so \
  -DTINYXML2_INCLUDE_DIR=/usr/include \
  -DTENSORRT_INCLUDE_DIR="$TRT_HEADERS" \
  -DTENSORRT_LIBRARY_INFER="$TRT_PREFIX/lib/libnvinfer.so" \
  -DTENSORRT_LIBRARY_INFER_PLUGIN="$TRT_PREFIX/lib/libnvinfer_plugin.so" \
  -DTENSORRT_LIBRARY_ONNXPARSER="$TRT_PREFIX/lib/libnvonnxparser.so"

set +u
source "$ROBOT/install/setup.bash"
set -u

if [[ ! -f "$ENGINE" ]]; then
  "$ROBOT/install/depth_anything_v3/lib/depth_anything_v3/generate_engines" "$MODEL_DIR"
fi

mkdir -p "$ROBOT/install/depth_anything_v3/share/depth_anything_v3/models"
ln -sfn "$ENGINE" "$ROBOT/install/depth_anything_v3/share/depth_anything_v3/models/$(basename "$ENGINE")"
echo "DA3 CUDA/TensorRT wrapper is ready. CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES"
