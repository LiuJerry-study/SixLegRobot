"""Run native motion checks and verify the browser calibration and replay fixtures."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def verify(path, expected, write):
    if write:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(expected, encoding="utf-8")
    elif not path.exists() or path.read_text(encoding="utf-8") != expected:
        raise SystemExit(f"Out of date: {path.relative_to(ROOT)}. Run test/check_motion.py --generate.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generate", action="store_true", help="update checked-in calibration and logs")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as temporary:
        binary = Path(temporary) / ("motion.exe" if os.name == "nt" else "motion")
        subprocess.run([
            os.environ.get("CXX", "g++"), "-std=c++11", "-Wall", "-Wextra", "-pedantic", "-Iinclude",
            "src/LegKinematics.cpp", "src/WalkMotion.cpp", "src/MotionFrame.cpp", "test/WalkMotionCheck.cpp",
            "-o", str(binary),
        ], cwd=ROOT, check=True)
        subprocess.run([binary], check=True, cwd=temporary)
        model = json.loads(subprocess.check_output([binary, "model"], text=True))
        js = (
            "// Generated from include/RobotModel.h by test/check_motion.py.\n"
            "(function (g) {\n  'use strict';\n  g.RobotModel = "
            + json.dumps(model, separators=(",", ":"))
            + ";\n})(typeof window !== 'undefined' ? window : globalThis);\n"
        )
        verify(ROOT / "gait-sim/js/robot-model.js", js, args.generate)
        for name in ("legacy", "three"):
            sample = subprocess.check_output([binary, name], cwd=temporary, text=True)
            verify(ROOT / "gait-sim/samples" / f"{name}.txt", sample, args.generate)
    subprocess.run(["node", "gait-sim/js/test-motion-frame.js"], cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
