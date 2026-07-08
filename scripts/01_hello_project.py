from pathlib import Path
import sys

project_root = Path(__file__).resolve().parents[1]

print("Hello, real_go2_control")
print("当前 Python 版本:", sys.version)
print("当前脚本位置:", Path(__file__).resolve())
print("项目根目录:", project_root)
