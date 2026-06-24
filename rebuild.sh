#!/bin/bash
# =============================================================================
# rebuild.sh — 清理并全量重新编译 (Python 统一在 robotdog conda 环境)
# 使用方法: bash rebuild.sh
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

echo "========================================"
echo "  机器人狗系统 — 重新编译"
echo "========================================"

# 1. 激活 robotdog conda 环境
source /home/ubuntu/bl/miniconda3/etc/profile.d/conda.sh
conda activate robotdog
echo "[INFO] Python: $(python --version)"
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++
echo "[INFO] C compiler: ${CC}"
echo "[INFO] C++ compiler: ${CXX}"

# 2. ROS2 Humble
source /opt/ros/humble/setup.bash

# 3. 清理
echo "[INFO] 清理旧的 build/install/log..."
rm -rf build/ install/ log/

# 4. 编译内置 DA3 C++ package 及其依赖
echo "[INFO] 准备 DA3 TensorRT C++ package..."
bash scripts/prepare_da3.sh

# 5. 编译其余 ROS2 packages
echo "[INFO] 开始编译..."
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++ \
  -DDEPTH_ANYTHING_V3_ENABLE_CUDA_KERNELS=ON \
  -DCMAKE_PREFIX_PATH="${CONDA_PREFIX};${CONDA_PREFIX}/targets/x86_64-linux;/opt/ros/humble;/usr;/usr/lib/x86_64-linux-gnu/cmake" \
  -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog \
  -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt \
  -Dconsole_bridge_DIR=/usr/lib/x86_64-linux-gnu/console_bridge/cmake \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DTINYXML2_LIBRARY=/usr/lib/x86_64-linux-gnu/libtinyxml2.so \
  -DTINYXML2_INCLUDE_DIR=/usr/include

echo ""
echo "========================================"
echo "  编译完成！启动方式:"
echo "    source setup_env.sh"
echo "    bash scripts/run_da3_video.sh"
echo "========================================"
