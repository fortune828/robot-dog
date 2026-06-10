#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WRAPPER="$ROOT/Depth/ros2-depth-anything-v3-trt"
ROBOT="$ROOT/robot-dog"
CONDA_PREFIX="/home/ubuntu/bl/miniconda3/envs/cv_deploy"
TRT_LIBS="$CONDA_PREFIX/lib/python3.9/site-packages/tensorrt_libs"
TRT_PREFIX="$ROOT/Depth/tensorrt-10.14-local"
VIDEO="${1:-$ROOT/robot-dog/data/videos/test_video.mp4}"
FOCAL="${2:-960.0}"

set +u
source /opt/ros/humble/setup.bash
set -u
set +u
source "$WRAPPER/install/setup.bash"
source "$ROBOT/install/setup.bash"
set -u
export CUDA_VISIBLE_DEVICES=1
export LD_LIBRARY_PATH="$TRT_PREFIX/lib:$CONDA_PREFIX/lib:$CONDA_PREFIX/targets/x86_64-linux/lib:$TRT_LIBS:${LD_LIBRARY_PATH:-}"
echo "Using physical GPU 1 as CUDA device 0; video=$VIDEO focal=${FOCAL}px"
exec ros2 launch sanitation_bringup da3_video_experiment.launch.py video_path:="$VIDEO" focal_length_px:="$FOCAL"
