#!/bin/bash
# =============================================================================
# setup_env.sh — 机器人狗 ROS2 环境设置 (Python 统一在 robotdog conda 环境)
# 使用方法: source setup_env.sh
# =============================================================================

# 1. 激活 robotdog conda 环境
if [ -f /home/ubuntu/bl/miniconda3/etc/profile.d/conda.sh ]; then
    source /home/ubuntu/bl/miniconda3/etc/profile.d/conda.sh
    conda activate robotdog
else
    echo "[ERROR] conda 未找到"
    return 1
fi

# 2. ROS2 Humble 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[ERROR] ROS2 Humble 未找到: /opt/ros/humble/setup.bash"
    return 1
fi

# 3. 工作空间环境
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOTDOG_ROOT="${_SCRIPT_DIR}"
export CUDA_HOME="${CONDA_PREFIX}"
export CUDA_PATH="${CONDA_PREFIX}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-1}"
export LD_LIBRARY_PATH="${_SCRIPT_DIR}/.local/tensorrt/lib:${CONDA_PREFIX}/lib:${CONDA_PREFIX}/targets/x86_64-linux/lib:${CONDA_PREFIX}/lib/python3.10/site-packages/tensorrt_libs:${LD_LIBRARY_PATH:-}"
if [ -f "${_SCRIPT_DIR}/install/setup.bash" ]; then
    source "${_SCRIPT_DIR}/install/setup.bash"
else
    echo "[WARN] 工作空间未编译，请先运行: bash rebuild.sh"
fi

echo "[INFO] 环境已就绪 | Python: $(python --version) | ROS2: Humble | CUDA: ${CUDA_HOME}"
echo "  启动全 Mock 系统:"
echo "    ros2 launch sanitation_bringup demo_mock_system.launch.py"
