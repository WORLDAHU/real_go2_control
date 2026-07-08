from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from real_go2_model import RealGo2Model


robot = RealGo2Model()
robot.print_summary()