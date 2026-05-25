"""pytest 全局配置"""
import sys
from pathlib import Path

# 将 src/ 下的包加入 sys.path，方便在无 ROS 环境下直接 import 测试
_src = Path(__file__).resolve().parent.parent / "src"
for _pkg in _src.iterdir():
    if _pkg.is_dir() and (_pkg / "setup.py").exists():
        _pkg_path = str(_pkg)
        if _pkg_path not in sys.path:
            sys.path.insert(0, _pkg_path)
