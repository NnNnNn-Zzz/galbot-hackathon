"""硬件访问层：stage 只认这一层，永远不直连 sdk_wrapper / galbot_sdk。

两个后端：
  WrapperIO —— 转发到 sdk_wrapper.wrapper，接口缺失时软降级（success=False + DEP_MISSING），
               不抛异常、不阻塞，符合 docs/DEVELOPMENT_RULES.md。
  MockIO    —— 纯内存仿真，不碰任何硬件。支持故障注入（失败/超时/不可判定/异常）。

切换点只有一处：场景入口 run(params) 里的 backend 选择。
    params["io"] = "wrapper" | "mock"
    或环境变量 MED_IO=mock
默认 wrapper。离线彩排必须显式 MED_IO=mock —— 因为 sdk_wrapper.MockRobot 只实现了 4 个
方法（init/set_base_pose/stop_base/get_sensor_data），其余全靠 WrapperIO 软降级，
那样只能测"依赖缺失"这一条路径，测不到真实成功路径。

所有 io 方法统一返回：
    {"success": bool, "message": str, ...附加字段}
失败时尽力带 "code": Code 里的字符串值。
"""

from __future__ import annotations

import logging
import math
import os
import time
from typing import Any, Dict, List, Optional, Tuple

from .contracts import Code

logger = logging.getLogger("scene.medicine.io")

#: 动作类方法白名单 —— stage 只允许调这些，别的都是越界。
IO_METHODS: Tuple[str, ...] = (
    "get_odom",
    "attach_tool",
    "set_arm_pose",
    "move_line_to",
    "set_joints",
    "set_gripper",
    "turn",
    "move_forward",
    "move_left",
    "move_to_pose",
    "speak",
    "capture_image",
    "is_holding",
    "fixture",
)


def _yaw_to_quat(yaw: float) -> List[float]:
    """yaw(rad) → 四元数 [x, y, z, w]，只绕 z。"""
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


class RobotIO:
    """硬件访问基类。子类必须实现全部白名单方法。

    返回契约（所有动作/感知方法共用，写死在这里便于队友对接）：
      success     bool   —— 调用是否成功
      message     str    —— 人类可读说明（失败原因写这里）
      code        str    —— 失败/不可判定时的原因码（contracts.Code 的 value）
      completion  str    —— "completed"=返回时动作**已完成**；
                           "dispatched"=仅**已下发**，完成要靠后续查询确认。
                           mock 全部是同步执行，恒为 "completed"；
                           真机（wrapper）由各接口实现时按 is_blocking 如实填写，
                           在 wrapper 落地前真机语义**未定**，调用方不得假设。
    """

    backend = "abstract"

    # ---------------------------------------------------------------- 元信息
    def available(self) -> Dict[str, bool]:
        """哪些接口是真能用的。用于开跑前的自检报告。"""
        out = {}
        for name in IO_METHODS:
            fn = getattr(self, name, None)
            out[name] = callable(fn) and not getattr(fn, "__isabstract__", False)
        return out

    # ---------------------------------------------------------------- 接口
    def get_odom(self) -> Dict[str, Any]:
        raise NotImplementedError

    def attach_tool(self, chain: str, tool: str) -> Dict[str, Any]:
        raise NotImplementedError

    def set_arm_pose(self, pose: List[float], timeout_s: float = 20.0) -> Dict[str, Any]:
        raise NotImplementedError

    def move_line_to(self, pose: List[float]) -> Dict[str, Any]:
        raise NotImplementedError

    def set_joints(self, positions: List[float], names: List[str],
                   speed_rad_s: float = 0.2) -> Dict[str, Any]:
        raise NotImplementedError

    def set_gripper(self, width_m: float, side: str = "left",
                    velocity_mps: float = 0.05, effort: float = 10.0) -> Dict[str, Any]:
        raise NotImplementedError

    def turn(self, deg: float) -> Dict[str, Any]:
        raise NotImplementedError

    def move_forward(self, distance_cm: float) -> Dict[str, Any]:
        raise NotImplementedError

    def move_left(self, distance_m: float) -> Dict[str, Any]:
        """底盘侧移（左 = base_link +y），朝向不变。单位：米。"""
        raise NotImplementedError

    def move_to_pose(self, x: float, y: float, yaw: float) -> Dict[str, Any]:
        raise NotImplementedError

    def speak(self, text: str) -> Dict[str, Any]:
        raise NotImplementedError

    def capture_image(self, tag: str = "") -> Dict[str, Any]:
        raise NotImplementedError

    def is_holding(self) -> Optional[bool]:
        """夹爪是否夹到东西。None = 本后端无此确证手段（禁止当成 True）。"""
        return None

    def fixture(self, name: str) -> Any:
        """离线固定数据（仅 mock 有），真机返回 None。"""
        return None


class WrapperIO(RobotIO):
    """转发到 sdk_wrapper。缺失接口一律软降级，不抛、不崩。"""

    backend = "wrapper"

    def __init__(self) -> None:
        self._wrapper = None
        self._missing: List[str] = []
        try:
            from sdk_wrapper import wrapper as _w  # 局部 import，mock 模式下不触发
            self._wrapper = _w
        except Exception as exc:  # noqa: BLE001 - 封装层坏掉也要能降级
            logger.error("导入 sdk_wrapper 失败：%s", exc)

    # ------------------------------------------------------------ 软调用核心
    def _call(self, name: str, *args: Any, **kwargs: Any) -> Dict[str, Any]:
        fn = getattr(self._wrapper, name, None) if self._wrapper is not None else None
        if fn is None:
            if name not in self._missing:
                self._missing.append(name)
            logger.warning("[依赖缺失] sdk_wrapper.%s 未实现（已记入缺口，见 to朱宁11.6.md）", name)
            return {"success": False, "message": f"sdk_wrapper.{name} 未实现",
                    "code": Code.DEP_MISSING.value}
        try:
            ret = fn(*args, **kwargs)
        except NotImplementedError as exc:
            if name not in self._missing:
                self._missing.append(name)
            logger.warning("[依赖缺失] sdk_wrapper.%s 抛 NotImplementedError: %s", name, exc)
            return {"success": False, "message": str(exc), "code": Code.DEP_MISSING.value}
        except Exception as exc:  # noqa: BLE001 - 单步失败不该炸掉整场
            logger.error("调用 sdk_wrapper.%s 失败: %s", name, exc)
            return {"success": False, "message": str(exc), "code": Code.STAGE_FAILED.value}
        if isinstance(ret, dict):
            ret.setdefault("success", bool(ret.get("success")))
            return ret
        return {"success": bool(ret), "message": str(ret)}

    def missing(self) -> List[str]:
        return list(self._missing)

    # ------------------------------------------------------------ 白名单实现
    def get_odom(self) -> Dict[str, Any]:
        return self._call("get_odom")

    def attach_tool(self, chain: str, tool: str) -> Dict[str, Any]:
        return self._call("attach_tool", chain=chain, tool=tool)

    def set_arm_pose(self, pose: List[float], timeout_s: float = 20.0) -> Dict[str, Any]:
        return self._call("set_arm_pose", pose, timeout_s)

    def move_line_to(self, pose: List[float]) -> Dict[str, Any]:
        return self._call("move_line_to", pose)

    def set_joints(self, positions: List[float], names: List[str],
                   speed_rad_s: float = 0.2) -> Dict[str, Any]:
        return self._call("set_joints", positions, names, speed_rad_s)

    def set_gripper(self, width_m: float, side: str = "left",
                    velocity_mps: float = 0.05, effort: float = 10.0) -> Dict[str, Any]:
        return self._call("set_gripper", width_m, side, velocity_mps, effort)

    def turn(self, deg: float) -> Dict[str, Any]:
        return self._call("turn", deg)

    def move_forward(self, distance_cm: float) -> Dict[str, Any]:
        return self._call("move_forward", distance_cm)

    def move_left(self, distance_m: float) -> Dict[str, Any]:
        return self._call("move_left", distance_m)

    def move_to_pose(self, x: float, y: float, yaw: float) -> Dict[str, Any]:
        return self._call("move_to_pose", x, y, yaw)

    def speak(self, text: str) -> Dict[str, Any]:
        return self._call("speak", text)

    def capture_image(self, tag: str = "") -> Dict[str, Any]:
        return self._call("capture_image", tag)


# ===========================================================================
# Mock
# ===========================================================================

class MockIO(RobotIO):
    """纯内存仿真：里程计、夹爪、桌面物体、语音全部在内存里推演。

    故障注入（阶段3 的异常覆盖靠它）：
        io.inject("set_gripper", "fail_once")      # 下一次夹爪动作失败
        io.inject("move_forward", "timeout", delay_s=0.2)
        io.inject("get_odom", "unknown")           # 返回不可判定
        io.inject("turn", "raise")                 # 抛异常，测 runner 兜底
    """

    backend = "mock"

    REACH_RADIUS_M = 0.09   # 夹爪闭合时能夹到的半径

    def __init__(self, objects: Optional[List[Dict[str, Any]]] = None) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.ee_pose: List[float] = [0.35, 0.0, 0.7, 0.0, 0.0, 0.0, 1.0]
        self.gripper_width = 0.10
        self.joints: List[float] = []
        self.tool = None
        self.holding: Optional[Dict[str, Any]] = None
        self.dropped: List[Dict[str, Any]] = []
        self.spoken: List[str] = []
        self.calls: List[str] = []
        self.table: List[Dict[str, Any]] = objects if objects is not None else default_table()
        self._faults: Dict[str, List[Dict[str, Any]]] = {}
        # 离线固定数据：让队友的 policy 在 mock 下也能拿到"标签文本"，
        # 真机上这里恒为空（fixture 返回 None），不允许拿它假冒识别结果。
        self._fixtures: Dict[str, Any] = {
            "objects": self.table,
            "label_text": {str(o.get("id")): str(o.get("expiry_date") or "") for o in self.table},
        }
        self.stopped = False

    # ------------------------------------------------------------ 故障注入
    def inject(self, method: str, behavior: str, times: int = 1,
               delay_s: float = 0.0, message: str = "") -> None:
        """给指定方法排队一个异常行为。

        behavior: fail | fail_once | timeout | unknown | raise
        """
        if behavior == "fail_once":
            behavior, times = "fail", 1
        self._faults.setdefault(method, []).append(
            {"behavior": behavior, "times": times, "delay_s": delay_s, "message": message})

    def clear_faults(self) -> None:
        self._faults.clear()

    def _take_fault(self, method: str) -> Optional[Dict[str, Any]]:
        queue = self._faults.get(method)
        if not queue:
            return None
        fault = queue[0]
        fault["times"] -= 1
        if fault["times"] <= 0:
            queue.pop(0)
        return fault

    def _guard(self, method: str) -> Optional[Dict[str, Any]]:
        """检查故障队列；命中则返回应直接返回的结果，否则 None。"""
        fault = self._take_fault(method)
        if fault is None:
            return None
        behavior = fault["behavior"]
        delay = float(fault.get("delay_s") or 0.0)
        if delay > 0:
            time.sleep(delay)
        msg = fault.get("message") or f"mock 注入：{method} {behavior}"
        if behavior == "raise":
            raise RuntimeError(msg)
        if behavior == "unknown":
            return {"success": False, "message": msg, "code": Code.UNDETERMINED.value}
        if behavior == "timeout":
            return {"success": False, "message": f"超时无回执：{msg}", "code": Code.TIMEOUT.value}
        return {"success": False, "message": msg, "code": Code.STAGE_FAILED.value}

    def fixture(self, name: str) -> Any:
        return self._fixtures.get(name)

    def set_fixtures(self, **kwargs: Any) -> None:
        self._fixtures.update(kwargs)

    # ------------------------------------------------------------ 白名单实现
    def get_odom(self) -> Dict[str, Any]:
        self.calls.append("get_odom")
        bad = self._guard("get_odom")
        if bad:
            return bad
        return {"success": True, "message": "ok",
                "odom": {"position": [self.x, self.y, 0.0],
                         "orientation": _yaw_to_quat(self.yaw)}}

    def attach_tool(self, chain: str, tool: str) -> Dict[str, Any]:
        self.calls.append("attach_tool")
        bad = self._guard("attach_tool")
        if bad:
            return bad
        self.tool = (chain, tool)
        return {"success": True, "message": f"已挂载 {tool} 到 {chain}"}

    def set_arm_pose(self, pose: List[float], timeout_s: float = 20.0) -> Dict[str, Any]:
        self.calls.append("set_arm_pose")
        bad = self._guard("set_arm_pose")
        if bad:
            return bad
        if len(pose) != 7:
            return {"success": False, "message": f"位姿应为 7 维，收到 {len(pose)} 维",
                    "code": Code.STAGE_FAILED.value}
        self.ee_pose = [float(v) for v in pose]
        return {"success": True, "message": "ok", "pose": list(self.ee_pose),
                "completion": "completed"}

    def move_line_to(self, pose: List[float]) -> Dict[str, Any]:
        self.calls.append("move_line_to")
        bad = self._guard("move_line_to")
        if bad:
            return bad
        if len(pose) != 7:
            return {"success": False, "message": f"位姿应为 7 维，收到 {len(pose)} 维",
                    "code": Code.STAGE_FAILED.value}
        self.ee_pose = [float(v) for v in pose]
        return {"success": True, "message": "ok", "pose": list(self.ee_pose),
                "completion": "completed"}

    def set_joints(self, positions: List[float], names: List[str],
                   speed_rad_s: float = 0.2) -> Dict[str, Any]:
        self.calls.append("set_joints")
        bad = self._guard("set_joints")
        if bad:
            return bad
        if names and len(names) != len(positions):
            return {"success": False,
                    "message": f"关节名 {len(names)} 个、位置 {len(positions)} 个，数量不一致",
                    "code": Code.STAGE_FAILED.value}
        self.joints = [float(v) for v in positions]
        return {"success": True, "message": "ok", "n": len(self.joints),
                "completion": "completed"}

    def set_gripper(self, width_m: float, side: str = "left",
                    velocity_mps: float = 0.05, effort: float = 10.0) -> Dict[str, Any]:
        self.calls.append("set_gripper")
        bad = self._guard("set_gripper")
        if bad:
            return bad
        self.gripper_width = float(width_m)
        if width_m < 0.05 and self.holding is None:
            grab = self._nearest_object()
            if grab is not None:
                self.holding = grab
                self.table = [o for o in self.table if o is not grab]
        elif width_m >= 0.05 and self.holding is not None:
            self.dropped.append(self.holding)
            self.holding = None
        return {"success": True, "message": "ok", "width_m": self.gripper_width,
                "holding": (self.holding or {}).get("id"), "completion": "completed"}

    def _nearest_object(self) -> Optional[Dict[str, Any]]:
        best, best_d = None, 1e9
        for obj in self.table:
            pos = obj.get("ee_pose") or obj.get("pose")
            if not pos:
                continue
            d = math.dist(list(pos[:3]), list(self.ee_pose[:3]))
            if d < best_d:
                best, best_d = obj, d
        if best is not None and best_d <= self.REACH_RADIUS_M:
            return best
        return None

    def turn(self, deg: float) -> Dict[str, Any]:
        self.calls.append("turn")
        bad = self._guard("turn")
        if bad:
            return bad
        self.yaw += math.radians(float(deg))
        return {"success": True, "message": "ok", "actual_angle_deg": float(deg),
                "completion": "completed"}

    def move_forward(self, distance_cm: float) -> Dict[str, Any]:
        self.calls.append("move_forward")
        bad = self._guard("move_forward")
        if bad:
            return bad
        d = float(distance_cm) / 100.0
        # 注意：wrapper 的 move_forward 用 abs() 吞负号，mock 这里如实按有符号推演，
        # 这样"后退变前进"这类 bug 在 mock 上就能暴露，不会被掩盖。
        self.x += d * math.cos(self.yaw)
        self.y += d * math.sin(self.yaw)
        return {"success": True, "message": "ok",
                "actual_distance_cm": float(distance_cm), "completion": "completed"}

    def move_left(self, distance_m: float) -> Dict[str, Any]:
        self.calls.append("move_left")
        bad = self._guard("move_left")
        if bad:
            return bad
        d = float(distance_m)
        # base_link 系：左 = +y。朝向 yaw 时，左侧方向 = yaw + 90°
        self.x += -d * math.sin(self.yaw)
        self.y += d * math.cos(self.yaw)
        return {"success": True, "message": "ok",
                "actual_distance_m": d, "completion": "completed"}

    def move_to_pose(self, x: float, y: float, yaw: float) -> Dict[str, Any]:
        self.calls.append("move_to_pose")
        bad = self._guard("move_to_pose")
        if bad:
            return bad
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        fx = c * float(x) - s * float(y)
        fy = s * float(x) + c * float(y)
        self.x += fx
        self.y += fy
        self.yaw += float(yaw)
        return {"success": True, "message": "ok",
                "position": [self.x, self.y, 0.0], "yaw": self.yaw,
                "completion": "completed"}

    def speak(self, text: str) -> Dict[str, Any]:
        self.calls.append("speak")
        bad = self._guard("speak")
        if bad:
            return bad
        if not text:
            return {"success": False, "message": "空文案", "code": Code.STAGE_FAILED.value}
        self.spoken.append(str(text))
        return {"success": True, "message": "ok", "played": True,
                "bytes": len(str(text).encode("utf-8")), "completion": "completed"}

    def capture_image(self, tag: str = "") -> Dict[str, Any]:
        self.calls.append("capture_image")
        bad = self._guard("capture_image")
        if bad:
            return bad
        return {"success": True, "message": "ok",
                "path": f"/tmp/mock_{tag or 'frame'}.jpg", "mock": True,
                "frame_id": "mock_frame_0", "completion": "completed"}

    def is_holding(self) -> Optional[bool]:
        return self.holding is not None

    # ------------------------------------------------------------ 自检辅助
    def stop_base(self) -> Dict[str, Any]:
        self.stopped = True
        return {"success": True, "message": "已停底盘"}

    def snapshot(self) -> Dict[str, Any]:
        return {"pose": [self.x, self.y, self.yaw], "holding": (self.holding or {}).get("id"),
                "dropped": [o.get("id") for o in self.dropped],
                "spoken": list(self.spoken), "calls": len(self.calls),
                "table_left": [o.get("id") for o in self.table]}


def default_table() -> List[Dict[str, Any]]:
    """默认桌面：三盒药，按 base 系 y 从大到小 = 从左到右。

    expiry_date 是给队友的过期判定用的**离线固定数据**，
    真机上日期要靠 OCR/VLM 自己读，不是 io 提供的。

    感知字段契约（队友交接三要素，详见 docs/handoff_medicine_baseline.md）：
      id         —— 实例编号（稳定唯一）
      image_ref  —— 图像关联：bbox 所在的帧标识（与 capture_image 返回的 frame_id 对应）
      bbox       —— 该帧内的像素框 [x, y, w, h]
      position_m —— 三维位置 [x, y, z]，单位米，坐标系 frame 字段指定（默认 base_link：
                     x 向前、y 向左、z 向上）。ee_pose 是历史别名，语义相同，新代码用 position_m
      expiry_date —— 仅 mock 固定数据；真机无此字段，读不到就是 None
    """
    return [
        {"id": "box_left", "label": "white", "ee_pose": [0.40, 0.30, 0.65],
         "position_m": [0.40, 0.30, 0.65], "frame": "base_link", "unit": "m",
         "image_ref": "mock_frame_0",
         "bbox": [0, 0, 60, 40], "score": 0.9, "expiry_date": "2026-08-01"},
        {"id": "box_mid", "label": "white", "ee_pose": [0.40, 0.10, 0.65],
         "position_m": [0.40, 0.10, 0.65], "frame": "base_link", "unit": "m",
         "image_ref": "mock_frame_0",
         "bbox": [80, 0, 60, 40], "score": 0.9, "expiry_date": "2027-12-01"},
        {"id": "box_right", "label": "white", "ee_pose": [0.40, -0.10, 0.65],
         "position_m": [0.40, -0.10, 0.65], "frame": "base_link", "unit": "m",
         "image_ref": "mock_frame_0",
         "bbox": [160, 0, 60, 40], "score": 0.9, "expiry_date": "2028-05-01"},
    ]


def build_io(backend: str = "", objects: Optional[List[Dict[str, Any]]] = None,
             faults: Optional[List[Dict[str, Any]]] = None) -> RobotIO:
    """唯一的后端选择点。上真机只改这里的默认值/参数，不动任何 stage。

    faults 仅对 mock 生效，格式：
        [{"method": "set_gripper", "behavior": "fail_once"},
         {"method": "move_forward", "behavior": "timeout", "delay_s": 0.2}]
    真机后端会忽略 faults（并在日志里提示），避免测试开关被误带进现场。
    """
    name = (backend or os.environ.get("MED_IO") or "wrapper").strip().lower()
    if name == "mock":
        logger.info("IO 后端 = mock（纯内存仿真，不接硬件）")
        io = MockIO(objects=objects)
        for spec in faults or []:
            io.inject(str(spec["method"]), str(spec.get("behavior", "fail_once")),
                      times=int(spec.get("times", 1)),
                      delay_s=float(spec.get("delay_s", 0.0)),
                      message=str(spec.get("message", "")))
        if faults:
            logger.warning("已注入 %d 条 mock 故障：%s", len(faults),
                           [f.get("method") for f in faults])
        return io
    if name != "wrapper":
        raise ValueError(f"未知 io 后端：{name}（只支持 wrapper | mock）")
    if faults:
        logger.warning("wrapper 后端忽略 mock_faults（%d 条），真机上不允许注入假故障", len(faults))
    logger.info("IO 后端 = wrapper（转发 sdk_wrapper）")
    return WrapperIO()
