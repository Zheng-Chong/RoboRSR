"""Run a RoboRSR Python entry with this package's G1-D geometry config for this process only.

Usage from the repository root:
    python output/dex1_camera_delivery_v1/code/runtime/launch.py scripts/g1d/scripted_demo.py ...
No project module or shared asset is installed/overwritten. Isaac imports remain in the target entry.
"""

import argparse
import importlib.abc
import importlib.util
import os
from pathlib import Path
import runpy
import sys


class DeliveryConfigFinder(importlib.abc.MetaPathFinder):
    def __init__(self, module_path):
        self.module_path = module_path

    def find_spec(self, fullname, path=None, target=None):
        if fullname == "env.robot_manager.robot_config.g1d":
            return importlib.util.spec_from_file_location(fullname, self.module_path)
        return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, default=Path(__file__).resolve().parents[2] / "model")
    parser.add_argument("script", type=Path)
    parser.add_argument("script_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not (args.asset_dir / "g1d_dex1.urdf").is_file():
        parser.error("Asset directory must contain the delivered g1d_dex1.urdf")
    os.environ["ROBORSR_G1D_ASSET_DIR"] = str(args.asset_dir.resolve())
    sys.meta_path.insert(0, DeliveryConfigFinder(Path(__file__).resolve().parent / "g1d.py"))
    script = args.script.resolve()
    sys.path[:0] = [str(script.parent), str(Path.cwd())]
    sys.argv = [str(script), *args.script_args]
    runpy.run_path(str(script), run_name="__main__")
