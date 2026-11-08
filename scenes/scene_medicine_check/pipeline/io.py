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
    # --- 2026-11-07 抽屉场景新增：三个**只读**示教接口 ---
    "get_ee_pose",
    "get_joint_positions",
    "get_gripper_width",
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

    def capture_image(self, tag: str = "", camera: str = "head_left") -> Dict[str, Any]:
        """拍一帧 RGB。camera: "head_left"/"head_right"（头部双目）/"left_arm"/"right_arm"。"""
        raise NotImplementedError

    # ------------------------------------------------- 只读：示教采点用（不发动作）
    def get_ee_pose(self, chain: str = "right_arm") -> Dict[str, Any]:
        """读末端在 base_link 下的当前位姿。

        返回 pose 为 [x, y, z, qx, qy, qz, qw]，单位 m。
        这是**只读**接口，示教时把手臂摆到位后调用即可，不会让任何关节动。
        """
        raise NotImplementedError

    def get_joint_positions(self, joint_groups: Optional[List[str]] = None,
                            joint_names: Optional[List[str]] = None) -> Dict[str, Any]:
        """读指定关节组的当前角度（弧度）。

        joint_groups 例：["right_arm"]。joint_names 传 [] 表示"整组都要"。
        ⚠️ 一定带上 joint_groups 且带 joint_names=[]，无参调用会返回 25 个错位数据（已踩过）。
        """
        raise NotImplementedError

    def get_gripper_width(self, side: str = "right") -> Dict[str, Any]:
        """读夹爪当前开度（米）。宽=张着，窄≈闭合，用于判断是否夹到东西。"""
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

    def capture_image(self, tag: str = "", camera: str = "head_left") -> Dict[str, Any]:
        return self._call("capture_image", tag, camera=camera)

    # ------------------------------------------------- 只读：示教采点
    def get_ee_pose(self, chain: str = "right_arm") -> Dict[str, Any]:
        # sdk_wrapper.get_ee_pose(chain) → {"pose": [7], "frame", "unit"}
        return self._call("get_ee_pose", chain=chain)

    def get_joint_positions(self, joint_groups: Optional[List[str]] = None,
                            joint_names: Optional[List[str]] = None) -> Dict[str, Any]:
        """转 sdk_wrapper.get_joint_positions（2026-11-07 补齐，返回 joint_names/joint_positions）。"""
        return self._call("get_joint_positions",
                          joint_groups=list(joint_groups or ["right_arm"]),
                          joint_names=list(joint_names or []))

    def get_gripper_width(self, side: str = "right") -> Dict[str, Any]:
        """转 sdk_wrapper.get_gripper_width（2026-11-07 补齐，返回 width）。"""
        return self._call("get_gripper_width", side=side)


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

    def __init__(self, objects: Optional[List[Dict[str, Any]]] = None,
                 start_joints: Optional[List[float]] = None,
                 start_pose: Optional[List[float]] = None,
                 teach_file: str = "teach_drawer.json") -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.ee_pose: List[float] = (list(start_pose) if start_pose and len(start_pose) == 7
                                     else [0.35, 0.0, 0.7, 0.0, 0.0, 0.0, 1.0])
        self.gripper_width = 0.10
        #: 起始关节角。None = 用构造值（下面的 get_joint_positions 默认值）。
        #: 这不是"真机数据"，只是让 mock 的初始态可以对齐某个示教点，
        #: 便于离线验证"从 A 回放到 B"这条链。
        self.joints: List[float] = [float(v) for v in (start_joints or [])]
        #: 分臂存关节角。原来只有一条 self.joints，导致 mock 下"读左臂"也会拿到右臂的值 ——
        #: home_to_carry.py 会把左臂误判成偏离 224°，真机上则是正常读数，这种差异
        #: 会让离线验证完全失去意义。键 = "right_arm" / "left_arm"。
        self.arm_joints: Dict[str, List[float]] = {}
        if self.joints:
            self.arm_joints["right_arm"] = list(self.joints)
        #: mock 的示教点查表文件（默认 teach_drawer.json，可由 build_io 覆盖）
        self.teach_file = str(teach_file)
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
        # ⚠️ 关节角也要跟着变。旧版只改 ee_pose 不改 joints，导致
        # 「拉完抽屉 → 读关节角」拿到的还是拉之前的值，下游的跨度检查会拿到
        # 一个 316° 的假差值。真机上 move_line 一定伴随关节运动。
        # mock 不做真 IK，用示教文件里最近的构型近似（带距离上限，够不着就保留原值）。
        approx = self._lookup_teach_pose_from_pose(self.ee_pose)
        if approx is not None:
            self.joints = approx
        return {"success": True, "message": "ok", "pose": list(self.ee_pose),
                "completion": "completed"}

    def _lookup_teach_pose_from_pose(self, pose: List[float]) -> Optional[List[float]]:
        """mock 专用：按末端位置在示教文件里找最近的点，返回它的关节角。

        只用**位置**（不用姿态），阈值 POS_MATCH_M。够不着就返回 None —— 
        宁可让 joints 保持不变（下游会看到"没动"），也不要硬套一个不相干的构型。
        """
        import json
        import os
        try:
            here = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(here, "configs", self.teach_file)
            if not os.path.exists(path):
                return None
            with open(path, "r", encoding="utf-8") as fh:
                pts = (json.load(fh).get("points") or {})
        except Exception:  # noqa: BLE001
            return None
        best, best_d = None, 1e9
        for rec in pts.values():
            p = rec.get("pose")
            jp = rec.get("joint_positions")
            if not p or len(p) != 7 or not jp or len(jp) != len(self.joints):
                continue
            d = math.dist(list(p[:3]), list(pose[:3]))
            if d < best_d:
                best, best_d = jp, d
        if best is None or best_d > float(getattr(self, "POS_MATCH_M", 0.03)):
            return None
        return [float(v) for v in best]

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
        _grp = MockIO._group_from_names(names) or "right_arm"
        self.arm_joints[_grp] = [float(v) for v in positions]
        # mock 的"到位"语义：不做真 FK，但要让后续 get_ee_pose 读得到**合理**的结果，
        # 否则"回放到示教点 → 以当前位姿为起点下探"这条链路在 mock 上走不通
        # （起点会被误判成初始位，安全闸门立即拦截，掩盖真正的逻辑问题）。
        # 做法：拿当前角去比对示教文件里的点位，最近的那个就直接取它的 pose。
        pose = self._lookup_teach_pose(positions)
        if pose is not None:
            self.ee_pose = list(pose)
        return {"success": True, "message": "ok", "n": len(self.joints),
                "completion": "completed"}

    def _lookup_teach_pose(self, positions: List[float]) -> Optional[List[float]]:
        """mock 专用：在示教文件里找最接近当前关节角的点，返回它的 pose。

        ⚠️ 这是**测试脚手架**，不是运动学。真机上永远走不到这条路。

        加了**距离上限** `max_dist`：关节角平方距离超过 1.0 rad²（≈ 单关节 57°）
        就当成"文件里没有这个姿态"，返回 None 而不是硬套一个不相干的 pose。
        否则下游会出现"回放到了 A 点，读出来的却是 B 点的位姿"这种假数据，
        比返回 None 更危险。
        """
        import json
        import os
        try:
            here = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(here, "configs", self.teach_file)
            if not os.path.exists(path):
                return None
            with open(path, "r", encoding="utf-8") as fh:
                pts = (json.load(fh).get("points") or {})
        except Exception:  # noqa: BLE001 - mock 助手出错不该影响主流程
            return None
        best, best_d = None, 1e9
        for rec in pts.values():
            jp = rec.get("joint_positions")
            if not jp or len(jp) != len(positions):
                continue
            d = sum((float(a) - float(b)) ** 2 for a, b in zip(jp, positions))
            if d < best_d:
                best, best_d = rec.get("pose"), d
        if best_d > 1.0:
            return None
        return [float(v) for v in best] if best and len(best) == 7 else None

    def set_gripper(self, width_m: float, side: str = "left",
                    velocity_mps: float = 0.05, effort: float = 10.0) -> Dict[str, Any]:
        """mock 夹爪语义（2026-11-08 修正）：

        旧版用 `width_m < 0.05` 判"闭爪"，结果 close_m=0.075（药盒 7.5 cm）时
        根本不进抓取分支 → 实验 3 在 mock 上永远空合，看起来像"没夹到"。
        这是 mock 的 bug，不是实验逻辑的问题。

        新语义贴近真机：
          · 指令宽度 < 全开值（GRIPPER_OPEN 0.10）→ 视为"在合爪"
          · 合爪时如果**末端附近有物体**，夹爪停在物体厚度处（宽度 = 物体厚度），
            并把物体记为已夹持；
          · 合爪时附近没东西 → 宽度 = 指令值（空合）；
          · 指令宽度 ≥ 全开值且手里有东西 → 松手，物体落到 dropped。
        """
        self.calls.append("set_gripper")
        bad = self._guard("set_gripper")
        if bad:
            return bad
        cmd = float(width_m)
        full_open = 0.10          # 与 tuning.GRIPPER_OPEN_M 一致；mock 内部常量，避免循环 import
        closed_cmd = cmd < full_open - 1e-6

        if closed_cmd and self.holding is None:
            grab = self._nearest_object()
            if grab is not None:
                self.holding = grab
                self.table = [o for o in self.table if o is not grab]
                # 夹住物体：宽度停在物体厚度，而不是指令值
                thickness = float(grab.get("width_m") or grab.get("thickness_m") or 0.0)
                self.gripper_width = thickness if thickness > 0 else cmd
            else:
                self.gripper_width = cmd
        elif (not closed_cmd) and self.holding is not None:
            self.dropped.append(self.holding)
            self.holding = None
            self.gripper_width = cmd
        else:
            self.gripper_width = cmd

        return {"success": True, "message": "ok", "width_m": self.gripper_width,
                "commanded_m": cmd,
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

    def capture_image(self, tag: str = "", camera: str = "head_left") -> Dict[str, Any]:
        self.calls.append("capture_image")
        bad = self._guard("capture_image")
        if bad:
            return bad
        return {"success": True, "message": "ok",
                "path": f"/tmp/mock_{camera}_{tag or 'frame'}.jpg", "mock": True,
                "camera": camera,
                "frame_id": f"mock_frame_{camera}_{tag or '0'}", "completion": "completed"}

    def is_holding(self) -> Optional[bool]:
        # ⚠️ 这个接口在真机 wrapper 上可能整个缺失（sdk_wrapper 目前就没有）。
        # 下游 s10_teach_drop 用 try/except 兜住，拿到 None 就报"不可判定"。
        # mock 这里也要能注入故障：raise → 下游 except 吃掉变 None；unknown → None。
        fault = self._take_fault("is_holding")
        if fault is not None:
            behavior = str(fault.get("behavior") or "")
            msg = fault.get("message") or f"mock 注入：is_holding {behavior}"
            if behavior == "raise":
                raise RuntimeError(msg)
            return None
        return self.holding is not None

    # ------------------------------------------------- 只读：示教采点
    def get_ee_pose(self, chain: str = "right_arm") -> Dict[str, Any]:
        self.calls.append("get_ee_pose")
        bad = self._guard("get_ee_pose")
        if bad:
            return bad
        return {"success": True, "message": "ok", "pose": list(self.ee_pose),
                "frame": "base_link", "unit": "m", "chain": chain}

    @staticmethod
    def _group_from_names(names: Optional[List[str]]) -> Optional[str]:
        """从 joint_names 里认出是哪条臂（'right_arm_joint3' → 'right_arm'）。"""
        for n in (names or []):
            for g in ("right_arm", "left_arm"):
                if str(n).startswith(g):
                    return g
        return None

    @staticmethod
    def _default_arm(group: str) -> List[float]:
        """构造用的默认关节角：右臂 = tuning.CARRY，左臂 = 其镜像。"""
        from . import tuning
        if group == "left_arm":
            return [float(v) for v in tuning.UPPER_JOINT_POSITIONS[2:9]]
        return [float(v) for v in tuning.CARRY_JOINT_POSITIONS]

    def get_joint_positions(self, joint_groups: Optional[List[str]] = None,
                            joint_names: Optional[List[str]] = None) -> Dict[str, Any]:
        """mock 下返回当前关节角；没有设置过就是一组构造值。

        ⚠️ 构造值（tuning.CARRY 及其左臂镜像）**不是真机示教值**，
        任何报告里都不许当成实测数据。要跑"从某个已知位出发"的链路，
        必须用 `build_io(..., start_joints=[...])` 显式给初值。
        ⚠️ 按臂分别返回：读左臂不会拿到右臂的值（2026-11-08 修）。
        """
        self.calls.append("get_joint_positions")
        bad = self._guard("get_joint_positions")
        if bad:
            return bad
        names = list(joint_names or [])
        group = self._group_from_names(names) or (list(joint_groups or ["right_arm"]))[0]
        if not names or len(names) != 7:
            names = [f"{group}_joint{i}" for i in range(1, 8)]
        # 优先用分臂存的；再退回 self.joints（右臂兼容）；最后退回构造值
        pos = self.arm_joints.get(group)
        if pos is None and group == "right_arm" and len(self.joints) == len(names):
            pos = self.joints
        if pos is None or len(pos) != len(names):
            pos = self._default_arm(group)
        return {"success": True, "message": "ok", "names": names,
                "joint_names": names,
                "joint_positions": [float(v) for v in pos], "mock": True}


    def get_gripper_width(self, side: str = "right") -> Dict[str, Any]:
        self.calls.append("get_gripper_width")
        bad = self._guard("get_gripper_width")
        if bad:
            return bad
        return {"success": True, "message": "ok", "width": float(self.gripper_width),
                "side": side, "holding": self.holding is not None}

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

    右臂作业区（2026-11-07 起全程右臂）：y 均为负值——base_link 的 y 轴
    指向机器人左侧。左起第一盒 box_left 位于 y=-0.10，与 tuning.GRASP_POSE 对齐。

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
        {"id": "box_left", "label": "white", "ee_pose": [0.40, -0.10, 0.75],
         "position_m": [0.40, -0.10, 0.75], "frame": "base_link", "unit": "m",
         "image_ref": "mock_frame_0",
         "bbox": [0, 0, 60, 40], "score": 0.9, "expiry_date": "2026-08-01"},
        {"id": "box_mid", "label": "white", "ee_pose": [0.40, -0.20, 0.75],
         "position_m": [0.40, -0.20, 0.75], "frame": "base_link", "unit": "m",
         "image_ref": "mock_frame_0",
         "bbox": [80, 0, 60, 40], "score": 0.9, "expiry_date": "2027-12-01"},
        {"id": "box_right", "label": "white", "ee_pose": [0.40, -0.30, 0.75],
         "position_m": [0.40, -0.30, 0.75], "frame": "base_link", "unit": "m",
         "image_ref": "mock_frame_0",
         "bbox": [160, 0, 60, 40], "score": 0.9, "expiry_date": "2028-05-01"},
    ]


def drawer_table() -> List[Dict[str, Any]]:
    """抽屉场景的 mock 桌面：一盒药**躺在抽屉里**，位置对齐 P3_dive_ref 的下探落点。

    ⚠️ 这是**离线仿真用的假数据**，不是实测。它存在的唯一理由是：
      实验 3 的 drawer_dive 从 P3_dive_ref（base 系 x=0.4785, y=-0.7949）竖直下探，
      落到 z≈0.825。如果 mock 桌面上没有东西恰好在那儿，
      `set_gripper` 就夹不到任何物体 → 如实报 NO_TARGET → 实验 3 在 mock 上永远跑不绿。
      为了能验证**管线**而不掩盖判据，这里放一个位置精确对齐的盒子。

    真机上这个位置来自视觉（YOLO 输出的盒中心）或示教点，绝不来自这张表。
    """
    return [
        {"id": "box_in_drawer", "label": "white",
         "ee_pose": [0.4785, -0.7949, 0.825],
         "position_m": [0.4785, -0.7949, 0.825],
         #: 药盒夹持方向厚度 8.2 cm —— 夹爪合到 7.5 cm 会停在这里，
         #: 于是 get_gripper_width 返回 0.082 > close_m(0.075)+3mm，判"夹到"。
         #: 没有这个字段时 mock 直接返回指令值，永远等于 close_m，会被判空合。
         "width_m": 0.082, "size_m": [0.082, 0.055, 0.022],
         "frame": "base_link", "unit": "m",
         "image_ref": "mock_frame_0",
         "bbox": [100, 80, 70, 45], "score": 0.92,
         "expiry_date": "2026-08-01"},
    ]


def drawer_p0_table() -> List[Dict[str, Any]]:
    """实验 2 的 mock 桌面：药盒在 **P0 正下方**的抽屉内腔里。

    ⚠️ 与 drawer_table() 一样是**离线仿真用的假数据**，不是实测。
      区别只在盒子位置：实验 2 从 `P0_measured` 竖直下探 42 cm（落点
      base 系 x=0.5251, y=-0.2162, z≈0.8248），所以盒子必须摆在那儿，
      否则 mock 上 `set_gripper` 夹不到东西 → 如实报 NO_TARGET →
      "运动链是否成立"这个本实验真正要答的问题会被 mock 的布局误差掩盖。

    盒厚取 **7.5 cm**：这正是用户 2026-11-08 指定的「咬合维持 7.5 cm 的夹爪距离」。
    合爪指令是 0.02 m（远小于盒厚），所以夹爪会停在 0.075 m —— 回读值 0.075 就是
    "夹到了"的证据；若盒不在那儿，回读会掉到 0.02，判定夹空。
    """
    return [
        {"id": "box_under_p0", "label": "white",
         "ee_pose": [0.5251, -0.2162, 0.8248],
         "position_m": [0.5251, -0.2162, 0.8248],
         #: 夹持方向厚度 7.5 cm（= 实验 2 要求维持的咬合距离）
         "width_m": 0.075, "size_m": [0.075, 0.055, 0.090],
         "frame": "base_link", "unit": "m",
         "image_ref": "mock_frame_0",
         "bbox": [120, 90, 70, 45], "score": 0.94,
         "expiry_date": "2026-09-15"},
    ]


#: mock 场景名 → 桌面构造函数。用 --objects 选。
SCENES = {
    "default": default_table,
    "drawer": drawer_table,
    "drawer_p0": drawer_p0_table,
}


def build_io(backend: str = "", objects: Optional[List[Dict[str, Any]]] = None,
             faults: Optional[List[Dict[str, Any]]] = None,
             start_joints: Optional[List[float]] = None,
             start_pose: Optional[List[float]] = None,
             teach_file: str = "teach_drawer.json") -> RobotIO:
    """唯一的后端选择点。上真机只改这里的默认值/参数，不动任何 stage。

    faults 仅对 mock 生效，格式：
        [{"method": "set_gripper", "behavior": "fail_once"},
         {"method": "move_forward", "behavior": "timeout", "delay_s": 0.2}]
    真机后端会忽略 faults（并在日志里提示），避免测试开关被误带进现场。

    start_joints / start_pose / teach_file 同样仅对 mock 生效：
      - start_joints：mock 的初始关节角（不设就用构造值）
      - start_pose：mock 的初始末端位姿
      - teach_file：mock 的 `_lookup_teach_pose()` 去哪个示教文件里找 pose
    真机后端会忽略它们（真机必须自己读，不允许用假初值）。
    """
    name = (backend or os.environ.get("MED_IO") or "wrapper").strip().lower()
    if name == "mock":
        logger.info("IO 后端 = mock（纯内存仿真，不接硬件）")
        io = MockIO(objects=objects, start_joints=start_joints,
                    start_pose=start_pose, teach_file=teach_file)
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
    for k, v in (("start_joints", start_joints), ("start_pose", start_pose)):
        if v is not None:
            logger.warning("wrapper 后端忽略 %s（真机初值必须自己读，不许用假值）", k)
    if faults:
        logger.warning("wrapper 后端忽略 mock_faults（%d 条），真机上不允许注入假故障", len(faults))
    logger.info("IO 后端 = wrapper（转发 sdk_wrapper）")
    return WrapperIO()
