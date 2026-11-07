import unittest
from types import SimpleNamespace
from medicine_frontend.drawer_sdk import RightArmSDK


class Params:
    def __getattr__(self, name):
        if name.startswith("set_"):
            return lambda value: setattr(self, name[4:], value)
        raise AttributeError(name)


class SDKTests(unittest.TestCase):
    def make(self, status="success"):
        self.calls = []
        def move(*args, **kwargs):
            self.calls.append((args, kwargs))
            return status
        def stop(group):
            self.calls.append(("stop", group))
            return "success"
        sdk = SimpleNamespace(Parameter=Params, MotionStatus=SimpleNamespace(SUCCESS="success"),
                              ControlStatus=SimpleNamespace(SUCCESS="success"))
        return RightArmSDK(SimpleNamespace(set_end_effector_pose=move),
                           SimpleNamespace(stop_controller=stop), sdk,
                           guard=lambda: True, target_factory=None)

    def test_single_arm_and_straight(self):
        adapter = self.make()
        self.assertTrue(adapter.move([0, 0, 0, 0, 0, 0, 1])["success"])
        args, kwargs = self.calls[0]
        self.assertEqual(args[1:3], ("right_arm", "base_link"))
        self.assertTrue(kwargs["params"].move_line)
        self.assertEqual(kwargs["params"].actuate, "with_chain_only")

    def test_failure_stops_and_latches(self):
        adapter = self.make("timeout")
        with self.assertRaises(RuntimeError):
            adapter.move([0, 0, 0, 0, 0, 0, 1])
        self.assertEqual(self.calls[-1], ("stop", "right_arm"))
        with self.assertRaises(RuntimeError):
            adapter.move([0, 0, 0, 0, 0, 0, 1])
        self.assertEqual(len(self.calls), 2)

    def test_guard_failure_sends_no_motion(self):
        adapter = self.make()
        adapter.guard = lambda: False
        with self.assertRaises(RuntimeError):
            adapter.move([0, 0, 0, 0, 0, 0, 1])
        self.assertEqual(self.calls, [])
