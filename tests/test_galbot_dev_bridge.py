"""Regression checks for the unmodified galbot_dev tool integration."""

import unittest
from unittest.mock import patch, sentinel

from sdk_wrapper.galbot_dev_bridge import create_toolkit


class GalbotDevBridgeTests(unittest.TestCase):
    def test_mock_toolkit_runs_without_sdk(self):
        kit = create_toolkit(mock=True)
        self.assertIs(kit.motion.safety, kit.navigation.safety)
        self.assertEqual(kit.perception.list_cameras().status.value, "succeeded")
        self.assertEqual(kit.motion.execute_gesture("wave_right").status.value, "running")

    def test_real_adapter_reuses_wrapper_instances_and_does_not_shut_them_down(self):
        kit = create_toolkit(mock=False, real_authorized=True)
        with patch("sdk_wrapper.galbot_dev_bridge.wrapper.get_motion", return_value=sentinel.motion) as motion, \
             patch("sdk_wrapper.galbot_dev_bridge.wrapper.get_nav", return_value=sentinel.nav) as nav, \
             patch("sdk_wrapper.galbot_dev_bridge.wrapper.get_robot", return_value=sentinel.robot) as robot, \
             patch("sdk_wrapper.galbot_dev_bridge.wrapper.shutdown") as shutdown:
            self.assertEqual(
                kit.adapter._ensure_interfaces(),
                (sentinel.robot, sentinel.motion, sentinel.nav),
            )
            self.assertEqual(
                kit.adapter._ensure_interfaces(),
                (sentinel.robot, sentinel.motion, sentinel.nav),
            )
            kit.adapter.close()
            shutdown.assert_not_called()
            motion.assert_called_once()
            nav.assert_called_once()
            robot.assert_called_once()

    def test_real_toolkit_stays_inert_without_authorization(self):
        kit = create_toolkit(mock=False)
        with patch("sdk_wrapper.galbot_dev_bridge.wrapper.get_robot") as robot:
            result = kit.motion.execute_gesture("wave_right")
            self.assertEqual(result.status.value, "rejected")
            robot.assert_not_called()


if __name__ == "__main__":
    unittest.main()
