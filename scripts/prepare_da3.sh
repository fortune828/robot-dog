#!/usr/bin/env bash
set -euo pipefail

ROBOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG="$ROBOT/src/depth_anything_v3"
MODEL_DIR="$ROBOT/models/da3"
TRT_HEADERS="$ROBOT/third_party/tensorrt/include"
TRT_PREFIX="$ROBOT/.local/tensorrt"
CUDA_PREFIX="$ROBOT/.local/cuda"
CONDA_PREFIX="${CV_DEPLOY_PREFIX:-/home/ubuntu/bl/miniconda3/envs/cv_deploy}"
TRT_LIBS="$CONDA_PREFIX/lib/python3.9/site-packages/tensorrt_libs"
MODEL="$MODEL_DIR/DA3METRIC-LARGE.onnx"

[[ -f "$MODEL" ]] || { echo "Missing model: $MODEL"; exit 1; }
[[ -f "$TRT_HEADERS/NvInfer.h" ]] || { echo "Missing vendored TensorRT headers: $TRT_HEADERS"; exit 1; }
[[ -x "$CONDA_PREFIX/bin/nvcc" ]] || { echo "Missing CUDA nvcc in cv_deploy"; exit 1; }
mkdir -p "$TRT_PREFIX/lib" "$CUDA_PREFIX/bin" "$CUDA_PREFIX/targets"
ln -sfn "$CONDA_PREFIX/bin/nvcc" "$CUDA_PREFIX/bin/nvcc"
ln -sfn "$CONDA_PREFIX/bin/crt" "$CUDA_PREFIX/bin/crt"
ln -sfn "$CONDA_PREFIX/targets/x86_64-linux/include" "$CUDA_PREFIX/include"
ln -sfn "$CONDA_PREFIX/targets/x86_64-linux" "$CUDA_PREFIX/targets/x86_64-linux"
ln -sfn "$CONDA_PREFIX/targets/x86_64-linux/lib" "$CUDA_PREFIX/lib64"
ln -sfn "$CONDA_PREFIX/nvvm" "$CUDA_PREFIX/nvvm"
# The wrapper has no .cu sources; old CMake CUDA-language probing breaks with conda CUDA.
if grep -q "^enable_language(CUDA)" "$PKG/CMakeLists.txt"; then
  sed -i "s/^enable_language(CUDA)$/# CUDA is linked as C++ runtime libraries; no CUDA language sources./" "$PKG/CMakeLists.txt"
fi
for lib in libnvinfer.so.10 libnvinfer_plugin.so.10 libnvonnxparser.so.10; do
  ln -sfn "$TRT_LIBS/$lib" "$TRT_PREFIX/lib/$lib"
  ln -sfn "$lib" "$TRT_PREFIX/lib/${lib%.10}"
done

set +u
source /opt/ros/humble/setup.bash
set -u
export PATH="$CONDA_PREFIX/nvvm/bin:$PATH"
export LD_LIBRARY_PATH="$TRT_PREFIX/lib:$CONDA_PREFIX/lib:$CONDA_PREFIX/targets/x86_64-linux/lib:$TRT_LIBS:${LD_LIBRARY_PATH:-}"
export CUDA_VISIBLE_DEVICES=1
export NVCC_PREPEND_FLAGS="-I$CONDA_PREFIX/targets/x86_64-linux/include -L$CONDA_PREFIX/targets/x86_64-linux/lib -L$CONDA_PREFIX/lib"
cd "$ROBOT"
colcon build --symlink-install --packages-select depth_anything_v3 --cmake-args \
  -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCUDA_TOOLKIT_ROOT_DIR="$CUDA_PREFIX" \
  -DTENSORRT_ROOT="$ROBOT/third_party/tensorrt" \
  -DTENSORRT_BUILD="$TRT_PREFIX"
set +u
source "$ROBOT/install/setup.bash"
set -u
ENGINE="$MODEL_DIR/DA3METRIC-LARGE.fp16-batch1.engine"
if [[ ! -f "$ENGINE" ]]; then
  "$ROBOT/install/depth_anything_v3/lib/depth_anything_v3/generate_engines" "$MODEL_DIR"
fi
mkdir -p "$ROBOT/install/depth_anything_v3/share/depth_anything_v3/models"
ln -sfn "$ENGINE" "$ROBOT/install/depth_anything_v3/share/depth_anything_v3/models/$(basename "$ENGINE")"
echo "Wrapper and FP16 engine are ready. Physical GPU 1 is visible as CUDA device 0."
