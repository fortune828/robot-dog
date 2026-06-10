#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WRAPPER="$ROOT/Depth/ros2-depth-anything-v3-trt"
PKG="$WRAPPER/depth_anything_v3"
ROBOT="$ROOT/robot-dog"
MODEL_DIR="$ROBOT/models/da3"
TRT_HEADERS="$ROOT/Depth/TensorRT-10.14-headers/include"
TRT_PREFIX="$ROOT/Depth/tensorrt-10.14-local"
CUDA_PREFIX="$ROOT/Depth/cuda-12.8-local"
CONDA_PREFIX="/home/ubuntu/bl/miniconda3/envs/cv_deploy"
TRT_LIBS="$CONDA_PREFIX/lib/python3.9/site-packages/tensorrt_libs"
MODEL="$MODEL_DIR/DA3METRIC-LARGE.onnx"

[[ -f "$MODEL" ]] || { echo "Missing model: $MODEL"; exit 1; }
[[ -x "$CONDA_PREFIX/bin/nvcc" ]] || { echo "Missing CUDA nvcc in cv_deploy"; exit 1; }
mkdir -p "$TRT_PREFIX/include" "$TRT_PREFIX/lib" "$CUDA_PREFIX/bin" "$CUDA_PREFIX/targets"
mkdir -p "$PKG/models"
ln -sfn "$MODEL" "$PKG/models/$(basename "$MODEL")"
if [[ -f "$MODEL_DIR/DA3METRIC-LARGE.fp16-batch1.engine" ]]; then
  ln -sfn "$MODEL_DIR/DA3METRIC-LARGE.fp16-batch1.engine" "$PKG/models/DA3METRIC-LARGE.fp16-batch1.engine"
fi
ln -sfn "$CONDA_PREFIX/bin/nvcc" "$CUDA_PREFIX/bin/nvcc"
ln -sfn "$CONDA_PREFIX/bin/crt" "$CUDA_PREFIX/bin/crt"
ln -sfn "$CONDA_PREFIX/targets/x86_64-linux/include" "$CUDA_PREFIX/include"
ln -sfn "$CONDA_PREFIX/targets/x86_64-linux" "$CUDA_PREFIX/targets/x86_64-linux"
ln -sfn "$CONDA_PREFIX/targets/x86_64-linux/lib" "$CUDA_PREFIX/lib64"
ln -sfn "$CONDA_PREFIX/nvvm" "$CUDA_PREFIX/nvvm"
cp -a "$TRT_HEADERS/." "$TRT_PREFIX/include/"
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
cd "$WRAPPER"
colcon build --symlink-install --cmake-args \
  -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCUDA_TOOLKIT_ROOT_DIR="$CUDA_PREFIX" \
  -DTENSORRT_ROOT="$TRT_PREFIX"
set +u
source "$WRAPPER/install/setup.bash"
set -u
ENGINE="$PKG/models/DA3METRIC-LARGE.fp16-batch1.engine"
if [[ ! -f "$ENGINE" ]]; then
  "$WRAPPER/install/depth_anything_v3/lib/depth_anything_v3/generate_engines" "$PKG/models"
fi
mkdir -p "$WRAPPER/install/depth_anything_v3/share/depth_anything_v3/models"
ln -sfn "$ENGINE" "$WRAPPER/install/depth_anything_v3/share/depth_anything_v3/models/$(basename "$ENGINE")"
echo "Wrapper and FP16 engine are ready. Physical GPU 1 is visible as CUDA device 0."
