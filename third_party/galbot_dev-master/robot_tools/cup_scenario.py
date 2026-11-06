"""robot_tools/cup_scenario.py —— "把不安全杯子推离桌沿" 的闭环编排。

════════════════════════════════════════════════════════════════════════════
为什么要有这一层（而不是让模型自己串工具）
════════════════════════════════════════════════════════════════════════════
这个场景是 **6–8 步的闭环**，而且每一步都要看上一次的结果再决定下一步。
让大模型现场即兴编排，任何一步理解偏了（比如该复测却没复测）演示就断。
所以把编排固化成**一个高层工具**，模型只需要说"处理一下桌上不安全的杯子"。

同时它仍然**只调工具、不碰 SDK**：
  * 推必须经过 ``MotionTool.push_once`` → ``RobotSafety`` 闸门；
  * 安全联锁（d 反而变小、推不动、次数用完）都在这里显式处理。

════════════════════════════════════════════════════════════════════════════
闭环逻辑
════════════════════════════════════════════════════════════════════════════
    inspect_table()
      ├─ 没有不安全杯子 → 播"一切正常"，不硬凑动作
      └─ 有 → speak(cup_unsafe)
            └─ 循环（≤ max_pushes）：
                 push_once(步长, 方向来自最新一次测量)
                 inspect_table() 复测
                   ├─ 所有杯子 d ≥ target      → 成功
                   ├─ 本步进展 < min_progress → 推不动 → 兜底/求助
                   └─ d 反而变小               → 方向错了 → 立即停
            └─ 成功 → speak(safe) + 庆祝动作
            └─ 失败 → speak(need_help)
"""

from __future__ import annotations

import json
import logging
import os
import typing

from contracts import ErrorCode, RunMode, TaskStatus, ToolResult, failed, ok, rejected
from robot_tools.audio import AudioTool
from robot_tools.motion import MotionTool

LOG = logging.getLogger(__name__)

__all__ = ["CupScenario", "DEFAULT_SCENARIO_CONFIG", "CUP_SCENARIO_TOOL"]

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
DEFAULT_SCENARIO_CONFIG = os.path.join(_CONFIG_DIR, "scenarios.json")

CUP_SCENARIO_TOOL = "run_cup_scenario"


class CupScenario:
    """把"判不安全 → 语音 → 小步推 → 复测 → 庆祝"固化成一个工具。"""

    TOOL_NAMES = (CUP_SCENARIO_TOOL,)

    def __init__(
        self,
        perception: typing.Any,
        audio: AudioTool,
        motion: MotionTool,
        *,
        config_path: str | os.PathLike[str] | None = None,
        config: dict[str, typing.Any] | None = None,
    ) -> None:
        self.perception = perception
        self.audio = audio
        self.motion = motion
        self.mode: RunMode = motion.mode
        if config is not None:
            self.config = config
        else:
            path = os.fspath(config_path) if config_path else DEFAULT_SCENARIO_CONFIG
            try:
                with open(path, encoding="utf-8") as fh:
                    self.config = json.load(fh)
            except Exception as exc:  # noqa: BLE001
                raise ValueError(f"读取场景配置失败 {path}: {exc}") from exc
        self.params: dict[str, typing.Any] = dict(self.config.get("push_cup") or {})
        if not self.params:
            raise ValueError("scenarios.json 里缺少 push_cup 段")

    # ------------------------------------------------------------------ #
    @property
    def _cfg(self) -> dict[str, typing.Any]:
        return self.params

    def _num(self, key: str, default: float) -> float:
        try:
            return float(self._cfg.get(key, default))
        except (TypeError, ValueError):
            return float(default)

    # ------------------------------------------------------------------ #
    def run_cup_scenario(self) -> ToolResult:
        """跑完整场景。返回的 ``data["trace"]`` 里有每一步的实测值，便于复盘。"""
        cfg = self._cfg
        camera_id = str(cfg.get("camera_id") or "left_arm")
        chain = str(cfg.get("chain") or "left_arm")
        gripper = str(cfg.get("gripper") or "left_gripper")
        target_m = self._num("target_m", 0.12)
        step_m = self._num("push_step_m", 0.025)
        max_pushes = int(self._num("max_pushes", 3))
        min_progress = self._num("min_progress_m", 0.005)
        prompts = cfg.get("prompts") or {}

        trace: list[dict[str, typing.Any]] = []
        warnings: list[str] = []

        def record(event: str, **payload: typing.Any) -> None:
            entry = {"event": event}
            entry.update(payload)
            trace.append(entry)

        # ── 1) 先看：有没有不安全的杯子 ────────────────────────────────
        first = self.perception.inspect_table(camera_id)
        if first.status is not TaskStatus.SUCCEEDED:
            record("inspect_failed", status=first.status.value, code=str(first.error_code))
            return ToolResult(
                status=first.status,
                mode=self.mode,
                message=f"桌面检测失败，未做任何动作：{first.message}",
                data={"trace": trace, "warnings": warnings},
                error_code=first.error_code,
            )
        record("inspect", cups=first.data.get("cups"), action=first.data.get("action"))

        if first.data.get("action") != "push":
            text_prompt = prompts.get("safe")
            if text_prompt:
                self.audio.speak(str(text_prompt))
            return ok(
                "所有杯子距桌沿都足够远，不需要处理",
                self.mode,
                data={"trace": trace, "cups": first.data.get("cups"), "action": "none"},
            )

        # ── 2) 语音告警 ───────────────────────────────────────────────
        if prompts.get("unsafe"):
            self.audio.speak(str(prompts["unsafe"]))
            record("speak", prompt_id=prompts["unsafe"])

        # ── 3) 闭环推 ─────────────────────────────────────────────────
        inspection = first
        baseline_d = self._min_distance(inspection)
        if baseline_d is None:
            return failed(
                ErrorCode.NOT_READY,
                "检测到不安全的杯子，但读不到它的距离；不做动作",
                self.mode,
                data={"trace": trace},
            )

        current_d = baseline_d
        pushes = 0
        outcome = "not_started"
        while pushes < max_pushes and current_d < target_m:
            remaining = max(0.0, target_m - current_d)
            step = min(step_m, max(0.005, remaining))
            direction = inspection.data.get("push_direction_base")

            push = self.motion.push_once(
                step,
                direction_base=direction,
                chain=chain,
                gripper=gripper,
                effort=self._num("gripper_effort", 10.0),
            )
            record(
                "push",
                index=pushes + 1,
                step_m=round(step, 4),
                direction=direction,
                status=push.status.value,
                code=str(push.error_code) if push.error_code else None,
            )
            if push.status is TaskStatus.UNKNOWN:
                # 机器人动没动不知道 → 立刻停手，交人工核实
                return ToolResult(
                    status=TaskStatus.UNKNOWN,
                    mode=self.mode,
                    message=(
                        f"推动过程中状态不可知（{push.message}）；"
                        f"不得认为机器人已停，请人工核实后再继续"
                    ),
                    data={"trace": trace, "last_known_d_m": round(current_d, 4)},
                    error_code=ErrorCode.STATE_UNCERTAIN,
                )
            if push.status is not TaskStatus.SUCCEEDED:
                outcome = "push_failed"
                warnings.append(f"第 {pushes + 1} 次推失败：{push.message}")
                break
            pushes += 1

            # 推完手臂就离开观察位姿了，而单应只在标定位姿下成立
            # → 必须**先回位，再复测**。这是固定单应方案的固有代价。
            observe_pose = inspection.data.get("observe_pose")
            if observe_pose:
                back = self.motion.move_ee_to(
                    observe_pose, chain=chain, straight_line=False
                )
                record("return_to_observe", status=back.status.value)
                if back.status is TaskStatus.UNKNOWN:
                    return ToolResult(
                        status=TaskStatus.UNKNOWN,
                        mode=self.mode,
                        message=(
                            f"回观察位姿时状态不可知（{back.message}）；"
                            f"不得认为机器人已停，请人工核实"
                        ),
                        data={"trace": trace},
                        error_code=ErrorCode.STATE_UNCERTAIN,
                    )
                if back.status is not TaskStatus.SUCCEEDED:
                    outcome = "return_failed"
                    warnings.append(f"回观察位姿失败：{back.message}")
                    break
            else:
                warnings.append("标定里没有 observe_pose，无法回位；复测可能因位姿偏差被拒")

            # 复测
            again = self.perception.inspect_table(camera_id)
            if again.status is not TaskStatus.SUCCEEDED:
                outcome = "recheck_failed"
                warnings.append(f"推完复测失败：{again.message}")
                break
            inspection = again
            new_d = self._min_distance(again)
            if new_d is None:
                outcome = "recheck_failed"
                warnings.append("复测读不到距离")
                break
            progress = new_d - current_d
            record("recheck", d_m=round(new_d, 4), progress_m=round(progress, 4))

            if new_d < current_d - 1e-9:
                # 距离反而变小：方向很可能反了，立即停（安全联锁）
                outcome = "wrong_direction"
                warnings.append(
                    f"复测发现 d 由 {current_d * 100:.1f} cm 变小到 {new_d * 100:.1f} cm，"
                    f"推的方向可能反了，已停止"
                )
                current_d = new_d
                break
            current_d = new_d
            if current_d >= target_m:
                outcome = "reached"
                break
            if progress < min_progress:
                outcome = "stuck"
                warnings.append(
                    f"本步只推进 {progress * 100:.1f} cm (< {min_progress * 100:.1f} cm)，"
                    f"判定推不动（可能需要改用抓取，或杯子太轻/桌面太滑）"
                )
                break
        else:
            outcome = "reached" if current_d >= target_m else "max_pushes"

        # ── 4) 收尾 ───────────────────────────────────────────────────
        success = current_d >= target_m
        payload: dict[str, typing.Any] = {
            "trace": trace,
            "warnings": warnings,
            "initial_d_m": round(baseline_d, 4),
            "final_d_m": round(current_d, 4),
            "target_m": target_m,
            "pushes": pushes,
            "outcome": outcome,
        }
        if success:
            if prompts.get("safe"):
                self.audio.speak(str(prompts["safe"]))
            gesture_id = cfg.get("celebrate_gesture")
            if gesture_id:
                celebration = self.motion.execute_gesture(str(gesture_id))
                payload["celebration"] = {
                    "gesture_id": gesture_id,
                    "status": celebration.status.value,
                    "code": str(celebration.error_code) if celebration.error_code else None,
                }
                if celebration.status is not TaskStatus.SUCCEEDED:
                    # 庆祝失败不影响任务成功，但要如实记下来
                    warnings.append(f"庆祝动作未执行：{celebration.message}")
            return ok(
                f"杯子已移到安全位置（距桌沿 {current_d * 100:.1f} cm ≥ "
                f"{target_m * 100:.0f} cm），推了 {pushes} 次",
                self.mode,
                data=payload,
            )

        if prompts.get("need_help"):
            self.audio.speak(str(prompts["need_help"]))
        return failed(
            ErrorCode.SAFETY_LIMIT if outcome != "push_failed" else ErrorCode.SDK_ERROR,
            f"未能把杯子移到安全区（当前 {current_d * 100:.1f} cm，目标 "
            f"{target_m * 100:.0f} cm，原因：{outcome}）；已停止并请求人工协助",
            self.mode,
            data=payload,
        )

    # ------------------------------------------------------------------ #
    @staticmethod
    def _min_distance(inspection: ToolResult) -> float | None:
        """取所有杯子里**最危险**的那个距离。全部达标才算成功。"""
        cups = (inspection.data or {}).get("cups") or []
        distances = [
            float(c["d_to_edge_m"])
            for c in cups
            if isinstance(c, dict) and c.get("d_to_edge_m") is not None
        ]
        return min(distances) if distances else None
