import importlib.util
import io
import unittest
from contextlib import redirect_stdout
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[3]
    / "scripts"
    / "test_charger_wrist_pregrasp.py"
)


def load_script():
    spec = importlib.util.spec_from_file_location(
        "test_charger_wrist_pregrasp",
        SCRIPT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ChargerWristScriptOutputTest(unittest.TestCase):
    def test_standard_tool_result_data_is_printed(self):
        script = load_script()
        output = io.StringIO()
        with redirect_stdout(output):
            script._print_result(
                "detect",
                {
                    "status": "succeeded",
                    "mode": "mock",
                    "message": "sampled",
                    "task_id": None,
                    "data": {
                        "eligible_for_precision": True,
                        "target_mode": "whole_mask_random_sample",
                        "whole_fallback_sampling": {
                            "sampled_uv": [320, 240],
                        },
                        "surface": {
                            "surface_center_m": [0.4, 0.1, 0.7],
                        },
                    },
                    "error_code": None,
                },
            )

        rendered = output.getvalue()
        self.assertIn("status=succeeded success=True", rendered)
        self.assertIn('"target_mode": "whole_mask_random_sample"', rendered)
        self.assertIn('"sampled_uv": [', rendered)


if __name__ == "__main__":
    unittest.main()
