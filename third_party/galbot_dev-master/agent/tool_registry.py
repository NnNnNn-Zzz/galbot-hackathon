"""agent/tool_registry.py —— Tool 的白名单注册表 + function-calling schema。

两条铁律（协议 §4 / §7）：
  1. Agent 只能调用**已注册**的工具。不在注册表里 = 模型说什么都调不到。
  2. 注册表里的 func 一律返回 `contracts.ToolResult`；
     `dispatch()` 负责转成 dict 再序列化成 JSON 字符串喂回模型。

为什么还要 schema 白名单而不是反射签名：
    反射出来的类型注解模型看不懂（`typing.Sequence[str]` 不是 JSON Schema），
    而且参数校验必须发生在**进工具之前**，否则坏参数已经打到硬件上了。
"""

from __future__ import annotations

import inspect
import json
import logging
import typing
from dataclasses import dataclass, field

from contracts import ErrorCode, RunMode, TaskStatus, ToolResult, rejected

__all__ = ["ToolSpec", "ToolRegistry", "build_default_registry"]

LOG = logging.getLogger(__name__)


@dataclass
class ToolSpec:
    name: str
    description: str
    parameters: dict[str, typing.Any]
    func: typing.Callable[..., ToolResult]
    owner: str = "B"  # 便于排查"这个工具是谁的"
    dangerous: bool = False  # 会驱动硬件的工具，默认要求显式授权

    def schema(self) -> dict[str, typing.Any]:
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": self.parameters,
            },
        }


def _validate_arguments(spec: ToolSpec, args: typing.Mapping[str, typing.Any]) -> str | None:
    """按 schema 做前置校验。返回错误说明，或 None 表示通过。

    为什么在 schema 上做而不是靠 Python 签名：
      * 模型给的是 JSON，类型/必填/多余字段必须按 JSON Schema 判；
      * 签名里的 TypeError 只能说明"调用不匹配"，说不出"缺了哪个必填字段"；
      * 最关键：校验必须在工具**执行前**完成，运动类工具尤其如此。
    """
    params = spec.parameters or {}
    required = params.get("required") or []
    missing = [key for key in required if key not in args]
    if missing:
        return f"缺少必填参数 {missing}；schema 要求 {required}"
    properties = params.get("properties") or {}
    if params.get("additionalProperties") is False:
        extra = [key for key in args if key not in properties]
        if extra:
            return f"出现未声明的参数 {extra}；允许的参数是 {sorted(properties)}"
    return None


@dataclass
class ToolRegistry:
    mode: RunMode = RunMode.MOCK
    _specs: dict[str, ToolSpec] = field(default_factory=dict)

    # ---- 注册 ----
    def register(self, spec: ToolSpec) -> None:
        if spec.name in self._specs:
            raise ValueError(f"工具 {spec.name!r} 重复注册")
        self._specs[spec.name] = spec

    def tool(
        self,
        name: str,
        description: str,
        parameters: dict[str, typing.Any] | None = None,
        *,
        owner: str = "B",
        dangerous: bool = False,
    ) -> typing.Callable[[typing.Callable[..., ToolResult]], typing.Callable[..., ToolResult]]:
        """装饰器式注册。"""

        def deco(func: typing.Callable[..., ToolResult]) -> typing.Callable[..., ToolResult]:
            self.register(
                ToolSpec(
                    name=name,
                    description=description,
                    parameters=parameters or {"type": "object", "properties": {}, "additionalProperties": False},
                    func=func,
                    owner=owner,
                    dangerous=dangerous,
                )
            )
            return func

        return deco

    # ---- 查询 ----
    @property
    def names(self) -> list[str]:
        return list(self._specs)

    def get(self, name: str) -> ToolSpec | None:
        return self._specs.get(name)

    def schemas(self) -> list[dict[str, typing.Any]]:
        return [spec.schema() for spec in self._specs.values()]

    def manifest(self) -> list[dict[str, typing.Any]]:
        return [
            {"name": s.name, "owner": s.owner, "dangerous": s.dangerous,
             "description": s.description.splitlines()[0][:80]}
            for s in self._specs.values()
        ]

    # ---- 分发 ----
    def dispatch(self, name: str, arguments: typing.Mapping[str, typing.Any] | str | None) -> ToolResult:
        """执行一个工具调用。任何异常都变成 ToolResult，绝不抛给 Agent 循环。"""
        spec = self._specs.get(name)
        if spec is None:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"未注册的工具 {name!r}；可用：{self.names}",
                self.mode,
                {"legal_tools": self.names},
            )
        args = arguments
        if isinstance(args, str):
            try:
                args = json.loads(args) if args.strip() else {}
            except json.JSONDecodeError as exc:
                return rejected(
                    ErrorCode.INVALID_ARGUMENT,
                    f"工具 {name!r} 的参数不是合法 JSON: {exc}",
                    self.mode,
                )
        if args is None:
            args = {}
        if not isinstance(args, typing.Mapping):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"工具 {name!r} 的参数必须是对象，收到 {type(args).__name__}",
                self.mode,
            )
        # 参数校验必须发生在**进工具之前**，否则坏参数已经打到硬件上了
        problem = _validate_arguments(spec, args)
        if problem is not None:
            return rejected(ErrorCode.INVALID_ARGUMENT, f"工具 {name!r} {problem}", self.mode,
                            {"tool": name, "received": sorted(args)})
        try:
            result = spec.func(**dict(args))
        except TypeError as exc:  # 参数名/个数不对
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"工具 {name!r} 调用签名不匹配: {exc}",
                self.mode,
                {"tool": name},
            )
        except Exception as exc:  # noqa: BLE001 - 工具边界兜底
            LOG.exception("tool %s crashed", name)
            return ToolResult(
                status=TaskStatus.FAILED,
                mode=self.mode,
                message=f"{type(exc).__name__}: {exc}",
                data={"tool": name},
                error_code=ErrorCode.SDK_ERROR,
            )
        if not isinstance(result, ToolResult):
            return ToolResult(
                status=TaskStatus.FAILED,
                mode=self.mode,
                message=f"工具 {name!r} 返回了 {type(result).__name__}，违反契约（必须是 ToolResult）",
                data={"tool": name},
                error_code=ErrorCode.SDK_ERROR,
            )
        return result

    def dispatch_json(self, name: str, arguments: typing.Mapping[str, typing.Any] | str | None) -> str:
        """给模型看的形态：紧凑 JSON 字符串。"""
        return json.dumps(self.dispatch(name, arguments).to_dict(), ensure_ascii=False)


# ═══════════════════════════════════════════════════════════════════════════ #
# B 侧：感知工具注册
# ═══════════════════════════════════════════════════════════════════════════ #
_CAMERA_ARG = {
    "type": "string",
    "description": "已登记的机位 ID。常用：left_arm（左臂，带深度）/ right_arm / head_left（头部广角，无深度）。留空用默认机位。",
}


def register_perception_tools(registry: ToolRegistry, perception: typing.Any) -> None:
    """把 PerceptionTool 的 public 方法注册进来。"""
    registry.register(
        ToolSpec(
            name="list_cameras",
            description=(
                "列出机器人上已登记的相机机位及其能力（是否有深度、适用距离）。"
                "在不确定该用哪路相机时先调用它。注意：登记 ≠ 本机已装配。"
            ),
            parameters={"type": "object", "properties": {}, "additionalProperties": False},
            func=perception.list_cameras,
            owner="B",
        )
    )
    registry.register(
        ToolSpec(
            name="capture_scene",
            description=(
                "抓取一帧图像并登记到帧缓存，返回 capture_id / rgb_ref / depth_ref。"
                "本工具**不返回像素**，只返回引用；只有需要留存证据或稍后复用时才单独调用，"
                "一般直接用 analyze_scene 或 find_object 即可。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "camera_id": _CAMERA_ARG,
                    "include_depth": {"type": "boolean", "description": "是否同时抓深度图，默认 true"},
                },
                "additionalProperties": False,
            },
            func=perception.capture_scene,
            owner="B",
        )
    )
    registry.register(
        ToolSpec(
            name="analyze_scene",
            description=(
                "抓一帧并做结构化分析：返回视野内的目标列表，每个目标含颜色标签、像素包围盒、"
                "中心像素、距离（米）和相机坐标系 3D 坐标。用于回答'你看到了什么/前面有什么'。"
                "结果里 depth_m 单位是米，center_cam_m 在 coordinate_frame 指定的坐标系中。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "camera_id": _CAMERA_ARG,
                    "want_labels": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": (
                            "只关心这些颜色/标签时填写，能显著减少耗时。"
                            "可选：red/orange/yellow/green/cyan/blue/purple/magenta/black/white/foreground。"
                            "留空则检测全部颜色。"
                        ),
                    },
                    "max_objects": {"type": "integer", "description": "最多返回几个目标，默认 8", "minimum": 1, "maximum": 20},
                },
                "additionalProperties": False,
            },
            func=perception.analyze_scene,
            owner="B",
        )
    )
    registry.register(
        ToolSpec(
            name="find_object",
            description=(
                "按颜色或'最近前景'检索单个目标，返回它的像素位置、距离（米）以及可选的 base_link 坐标。"
                "这是'红色方块在哪 / 离我多远'这类问题的首选工具。"
                "找不到会返回 status=failed，error_code=not_ready（不代表出错，是视野内确实没有）。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "label": {
                        "type": "string",
                        "description": "目标标签：颜色名（red/blue/绿色…）或 foreground（最近的前景物体）",
                    },
                    "camera_id": _CAMERA_ARG,
                    "max_distance_m": {
                        "type": "number",
                        "description": "只考虑这个距离（米）以内的目标，默认 1.5",
                        "minimum": 0.05,
                        "maximum": 10.0,
                    },
                    "with_base_link": {
                        "type": "boolean",
                        "description": "是否把相机系坐标换算到 base_link（需要外参可用），默认 true",
                    },
                },
                "required": ["label"],
                "additionalProperties": False,
            },
            func=perception.find_object,
            owner="B",
        )
    )
    registry.register(
        ToolSpec(
            name="measure_distance",
            description=(
                "测量指定像素处的真实距离（米），内部会取该像素邻域的中位数以抗噪。"
                "仅在 analyze_scene 已给出像素坐标、但你需要复核某一点距离时使用。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "u": {"type": "integer", "description": "像素列坐标（x），从 0 开始"},
                    "v": {"type": "integer", "description": "像素行坐标（y），从 0 开始"},
                    "camera_id": _CAMERA_ARG,
                },
                "required": ["u", "v"],
                "additionalProperties": False,
            },
            func=perception.measure_distance,
            owner="B",
        )
    )
    registry.register(
        ToolSpec(
            name="inspect_table",
            description=(
                "判断桌面上每个杯子到桌沿的距离（米），标出 C1/C2，并给出"
                "**要不要推、往哪个方向推、推多远**。"
                "这是'把不安全的杯子推到安全区'场景的第一步。"
                "需要赛前做好桌面标定，未标定会返回 not_ready。"
                "data.action: 'push' 需处理 / 'none' 无需处理 / 'blocked' 缺标定。"
            ),
            parameters={
                "type": "object",
                "properties": {"camera_id": _CAMERA_ARG},
                "additionalProperties": False,
            },
            func=perception.inspect_table,
            owner="B",
        )
    )


# ═══════════════════════════════════════════════════════════════════════════ #
# A 侧：执行类工具注册（共用同一个 RobotSafety）
# ═══════════════════════════════════════════════════════════════════════════ #
_A_MODULES = ("robot_tools.state", "robot_tools.motion", "robot_tools.navigation", "robot_tools.safety")


#: 协议 §3 冻结的 5 个执行类 Tool。名字与参数名不得改；
#: schema 写在 B 这边，这样 A 不必为了接入 Agent 再写一份 JSON Schema。
_PEER_TOOL_SCHEMAS: dict[str, dict[str, typing.Any]] = {
    "get_robot_state": {
        "description": (
            "读取机器人当前状态：是否连上、运动状态（idle/moving/unknown）、正在运行的任务 ID 等。"
            "status=unknown 时不得认为机器人已停止。"
        ),
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    "execute_gesture": {
        "description": (
            "执行一个**已登记验收过**的预设动作（不接受任意关节角/速度/力矩）。"
            "返回 running + task_id 时只代表已下发，必须用 get_task_status 轮询到 succeeded。"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "gesture_id": {
                    "type": "string",
                    "description": "白名单动作 ID（如 wave_right）；未登记的会被 rejected + invalid_argument",
                }
            },
            "required": ["gesture_id"],
            "additionalProperties": False,
        },
    },
    "navigate_to_place": {
        "description": (
            "导航到**已登记**的地点。返回 running 只代表任务已下发，**不代表已到达**；"
            "必须轮询 get_task_status 直到 succeeded 才能说到达。"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "place_id": {
                    "type": "string",
                    "description": "白名单地点 ID；未登记的会被 rejected + invalid_argument",
                }
            },
            "required": ["place_id"],
            "additionalProperties": False,
        },
    },
    "get_task_status": {
        "description": (
            "查询导航/动作任务的真实状态。succeeded 才算完成；running 还在跑；"
            "unknown 表示状态不可知（如控制中断网），必须暂停后续运动并交人工核实。"
        ),
        "parameters": {
            "type": "object",
            "properties": {"task_id": {"type": "string", "description": "启动任务时返回的 task_id"}},
            "required": ["task_id"],
            "additionalProperties": False,
        },
    },
    "cancel_task": {
        "description": (
            "请求取消一个任务。只有 status=cancelled 才代表**已确认停止**；"
            "返回 succeeded/running 都不能推断物理已停。硬急停是独立实体设施。"
        ),
        "parameters": {
            "type": "object",
            "properties": {"task_id": {"type": "string", "description": "要取消的 task_id"}},
            "required": ["task_id"],
            "additionalProperties": False,
        },
    },
    # ---- 末端笛卡尔原语（推/抓 都用这三个）----
    "get_end_effector_pose": {
        "description": "读末端在 base_link 下的位姿 [x,y,z,qx,qy,qz,qw]（米、四元数）。",
        "parameters": {
            "type": "object",
            "properties": {
                "chain": {
                    "type": "string",
                    "description": "手臂链",
                    "enum": ["left_arm", "right_arm"],
                }
            },
            "additionalProperties": False,
        },
    },
    "set_gripper": {
        "description": (
            "设置夹爪开度（米）。推杯子时闭到 0 用夹爪侧面推；"
            "抓取时闭到杯径（约 0.05–0.08）。开度必须在 [0, 0.11] 米内。"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "end_effector": {"type": "string", "enum": ["left_gripper", "right_gripper"]},
                "width_m": {"type": "number", "minimum": 0.0, "maximum": 0.11},
                "effort": {"type": "number", "minimum": 1.0, "maximum": 100.0},
            },
            "required": ["width_m"],
            "additionalProperties": False,
        },
    },
    "push_once": {
        "description": (
            "沿指定方向**直线**推一小段（单次上限 5 cm），推之前会自动闭合夹爪。"
            "小步 + 复测才可控：推完必须用 inspect_table 重新测量距离，"
            "不要一次推 10 cm。direction_base 优先于 direction。"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "distance_m": {"type": "number", "minimum": 0.005, "maximum": 0.05},
                "direction": {
                    "type": "string",
                    "enum": ["x+", "x-", "y+", "y-", "z+", "z-"],
                    "description": "base_link 下的方向；x+ 是远离机器人",
                },
                "direction_base": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "直接用 3 维向量（来自 inspect_table 的 push_direction_base）",
                },
                "chain": {"type": "string", "enum": ["left_arm", "right_arm"]},
            },
            "required": ["distance_m"],
            "additionalProperties": False,
        },
    },
}

#: A 的 adapter 必须实现的执行类方法；缺任何一个就说明它不是执行层 adapter
#: （例如 B 的离线合成场景 adapter），此时不注册 A 侧工具，避免造出一堆必然失败的 Tool。
_REQUIRED_ADAPTER_METHODS = (
    "get_robot_state",
    "execute_gesture",
    "navigate_to_place",
    "get_task_status",
    "cancel_task",
)


def _accepts_kwarg(target: typing.Any, name: str) -> bool:
    """探测 target 的构造签名是否接受某个关键字参数。"""
    try:
        return name in inspect.signature(target).parameters
    except (TypeError, ValueError):
        return False


def is_execution_adapter(adapter: typing.Any) -> bool:
    """adapter 是否具备执行层能力（5 个方法都在）。"""
    return all(callable(getattr(adapter, method, None)) for method in _REQUIRED_ADAPTER_METHODS)


def _build_shared_safety(adapter: typing.Any, real_authorized: bool) -> typing.Any:
    """建**一个** RobotSafety 给所有执行类工具共用。

    为什么必须共用：RobotSafety 内部持有任务表、单任务互斥和"不确定态熔断"。
    每个 Tool 各建一个，就会出现「motion 启动的任务 navigation 查不到」，
    以及熔断只对观察到它的那个实例生效 —— 护栏形同虚设。
    """
    try:
        from robot_tools.safety import RobotSafety
    except ImportError:
        LOG.debug("robot_tools.safety 不可用，A 侧工具将各自建 safety")
        return None
    except Exception as exc:  # noqa: BLE001 - A 的代码有 bug 不该拖垮 B
        LOG.warning("导入 robot_tools.safety 失败：%s", exc)
        return None

    kwargs: dict[str, typing.Any] = {}
    if real_authorized and _accepts_kwarg(RobotSafety, "real_authorized"):
        kwargs["real_authorized"] = True
    try:
        return RobotSafety(adapter, **kwargs)
    except Exception as exc:  # noqa: BLE001
        LOG.warning("RobotSafety 构造失败：%s", exc)
        return None


def _instantiate_peer(
    cls: typing.Any, adapter: typing.Any, safety: typing.Any, real_authorized: bool
) -> typing.Any:
    """实例化 A 的 Tool 类，尽量把共享 safety 传进去；签名不符就退回最少假设。"""
    kwargs: dict[str, typing.Any] = {}
    if safety is not None and _accepts_kwarg(cls, "safety"):
        kwargs["safety"] = safety
    if real_authorized and _accepts_kwarg(cls, "real_authorized"):
        kwargs["real_authorized"] = True
    try:
        return cls(adapter, **kwargs)
    except TypeError as exc:
        LOG.debug("%s 构造签名不符(%s)，退回 cls(adapter)", getattr(cls, "__name__", cls), exc)
    except Exception as exc:  # noqa: BLE001
        LOG.warning("%s 实例化失败：%s", getattr(cls, "__name__", cls), exc)
        return None
    try:
        return cls(adapter)
    except Exception as exc:  # noqa: BLE001
        LOG.warning("%s 实例化失败：%s", getattr(cls, "__name__", cls), exc)
        return None


def register_peer_tools(
    registry: ToolRegistry,
    adapter: typing.Any,
    *,
    real_authorized: bool = False,
    handles: dict[str, typing.Any] | None = None,
) -> list[str]:
    """注册 A 侧执行类 Tool。

    三级优先，互不重复：
      1. 模块级钩子 `register_tools(registry, adapter)` —— A 显式接管（最高优先）
      2. 类上的 `TOOL_NAMES` —— A 自己声明要暴露哪些方法
      3. 协议 §3 冻结的 5 个工具名 —— B 侧兜底绑定，A 什么都不用改

    全部执行类工具**共用同一个 RobotSafety**，保证任务表与安全护栏是同一份。
    """
    import importlib

    if adapter is None or not is_execution_adapter(adapter):
        LOG.debug("当前 adapter 不是执行层适配器，跳过 A 侧工具注册")
        return []

    before = set(registry.names)

    # ── 第 1 级：模块级钩子 ─────────────────────────────────────────── #
    peer_classes: dict[str, typing.Any] = {}
    for module_name in _A_MODULES:
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            LOG.debug("A 侧模块 %s 尚未就绪，跳过", module_name)
            continue
        except Exception as exc:  # noqa: BLE001 - A 的代码有 bug 不该拖垮 B
            LOG.warning("导入 %s 失败：%s", module_name, exc)
            continue

        hook = getattr(module, "register_tools", None)
        if callable(hook):
            try:
                hook(registry, adapter)
            except Exception as exc:  # noqa: BLE001
                LOG.warning("%s.register_tools() 抛错：%s", module_name, exc)

        # 记下按命名约定暴露的类（第 2/3 级会用到）
        class_name = module_name.rsplit(".", 1)[-1].capitalize() + "Tool"
        cls = getattr(module, class_name, None)
        if cls is not None:
            peer_classes[class_name] = cls

    # ── 共享 safety + 实例化 ────────────────────────────────────────── #
    safety = _build_shared_safety(adapter, real_authorized)
    instances: dict[str, typing.Any] = {}
    for class_name, cls in peer_classes.items():
        instance = _instantiate_peer(cls, adapter, safety, real_authorized)
        if instance is not None:
            instances[class_name] = instance

    # 把创建出来的对象交给调用方复用，避免别处再 new 一个 RobotSafety
    if handles is not None:
        handles.update(
            {
                "safety": safety,
                "instances": instances,
                "motion": instances.get("MotionTool"),
                "navigation": instances.get("NavigationTool"),
                "state": instances.get("StateTool"),
            }
        )

    # ── 第 3 级：协议兜底绑定 ──────────────────────────────────────── #
    # get_robot_state 没有专用 Tool 类时直接绑 adapter（只读，不经过 safety）；
    # 其余四个**只**绑 Tool 类，避免绕过 safety 的任务追踪与护栏。
    bindings: dict[str, typing.Any] = {
        "get_robot_state": instances.get("StateTool") or adapter,
        "execute_gesture": instances.get("MotionTool"),
        "navigate_to_place": instances.get("NavigationTool"),
        "get_task_status": instances.get("NavigationTool"),
        "cancel_task": instances.get("NavigationTool"),
    }
    for name, owner_obj in bindings.items():
        if owner_obj is None or registry.get(name) is not None:
            continue
        func = getattr(owner_obj, name, None)
        if not callable(func):
            continue
        schema = _PEER_TOOL_SCHEMAS[name]
        registry.register(
            ToolSpec(
                name=name,
                description=schema["description"],
                parameters=schema["parameters"],
                func=func,
                owner="A",
                dangerous=name != "get_robot_state",
            )
        )

    # ── 第 2 级：A 自己声明的额外工具 ──────────────────────────────── #
    for instance in instances.values():
        for name in getattr(instance, "TOOL_NAMES", ()) or ():
            if registry.get(name) is not None:
                continue
            func = getattr(instance, name, None)
            if not callable(func):
                continue
            schema = _PEER_TOOL_SCHEMAS.get(name)
            registry.register(
                ToolSpec(
                    name=name,
                    description=schema["description"] if schema else f"[A] {name}",
                    parameters=(
                        schema["parameters"]
                        if schema
                        else {"type": "object", "properties": {}, "additionalProperties": True}
                    ),
                    func=func,
                    owner="A",
                    dangerous=name != "get_robot_state",
                )
            )

    return sorted(set(registry.names) - before)


def register_cup_scenario_tools(
    registry: ToolRegistry,
    perception: typing.Any,
    motion: typing.Any,
    *,
    config_path: str | None = None,
) -> list[str]:
    """注册"推杯子"场景需要的两个工具：``speak`` 和 ``run_cup_scenario``。

    复用注册表里那**同一个** MotionTool（也就复用了同一个 RobotSafety），
    不能再 new 一个，否则安全护栏会被复制成两份。
    """
    from robot_tools.audio import PROMPTS, AudioTool
    from robot_tools.cup_scenario import CUP_SCENARIO_TOOL, CupScenario

    audio = AudioTool(motion.safety.adapter, mode=motion.mode)
    scenario = CupScenario(perception, audio, motion, config_path=config_path)

    if registry.get("speak") is None:
        registry.register(
            ToolSpec(
                name="speak",
                description=(
                    "播放一条预录语音提示（只能选白名单里的 prompt_id）。"
                    "素材缺失时会退回把文案打到终端，并在 data.mode 标成 text_fallback。"
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "prompt_id": {
                            "type": "string",
                            "enum": sorted(PROMPTS),
                            "description": "cup_unsafe=检测到杯子靠近桌沿 / moving / safe / need_help",
                        }
                    },
                    "required": ["prompt_id"],
                    "additionalProperties": False,
                },
                func=audio.speak,
                owner="B",
            )
        )
    if registry.get(CUP_SCENARIO_TOOL) is None:
        registry.register(
            ToolSpec(
                name=CUP_SCENARIO_TOOL,
                description=(
                    "端到端执行'把不安全的杯子推离桌沿'：判距离 → 语音告警 → 小步推 → "
                    "每步复测 → 达标后语音+庆祝；推不动/方向错/次数用完会停下来喊人。"
                    "内部已含安全联锁，一次调用即可完成任务，不需要手动串工具。"
                ),
                parameters={"type": "object", "properties": {}, "additionalProperties": False},
                func=scenario.run_cup_scenario,
                owner="B",
                dangerous=True,
            )
        )
    return ["speak", CUP_SCENARIO_TOOL]


def build_default_registry(
    perception: typing.Any | None = None,
    adapter: typing.Any = None,
    mode: RunMode = RunMode.MOCK,
    include_peer: bool = True,
    real_authorized: bool = False,
    include_cup_scenario: bool = True,
    scenario_config: str | None = None,
) -> ToolRegistry:
    registry = ToolRegistry(mode=mode)
    if perception is not None:
        register_perception_tools(registry, perception)

    handles: dict[str, typing.Any] = {}
    if include_peer:
        found = register_peer_tools(
            registry, adapter, real_authorized=real_authorized, handles=handles
        )
        if found:
            LOG.info("已注册 A 侧工具: %s", found)
        else:
            LOG.debug("未注册任何 A 侧工具（adapter 非执行层，或 A 尚未就绪）")

    # 场景工具依赖「同一个」MotionTool（= 同一个 RobotSafety）。没有就跳过。
    motion = handles.get("motion")
    if include_cup_scenario and perception is not None and motion is not None:
        try:
            added = register_cup_scenario_tools(
                registry, perception, motion, config_path=scenario_config
            )
            LOG.info("已注册场景工具: %s", added)
        except Exception as exc:  # noqa: BLE001 - 场景工具装不上不该拖垮整个 Agent
            LOG.warning("注册推杯子场景工具失败：%s", exc)
    return registry
