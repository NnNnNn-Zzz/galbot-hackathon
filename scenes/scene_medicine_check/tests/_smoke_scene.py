"""临时联调脚本：验证 scene.py 的 table 分支在改造后仍可用（可删）。"""

import logging
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

logging.basicConfig(level=logging.ERROR, format="%(levelname)s %(name)s: %(message)s")

from scenes.scene_medicine_check import scene

r = scene.run({"mode": "table", "dry_run": True})
print("RESULT table_dry_run:", r.get("success"), "|", r.get("feedback"))

r2 = scene.run({"mode": "table", "io": "mock", "pause_s": 0})
print("RESULT table_mock:", r2.get("success"), "|", r2.get("feedback"))
print("RESULT picked:", (r2.get("details") or {}).get("picked"))
sys.exit(0)
