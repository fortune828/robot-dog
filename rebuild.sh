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

# 2. ROS2 Humble
source /opt/ros/humble/setup.bash

# 3. 清理
echo "[INFO] 清理旧的 build/install/log..."
rm -rf build/ install/ log/

# 4. 编译
echo "[INFO] 开始编译..."
colcon build --symlink-install

echo ""
echo "========================================"
echo "  编译完成！启动方式:"
echo "    source setup_env.sh"
echo "    bash scripts/run_da3_video.sh"
echo "========================================"
