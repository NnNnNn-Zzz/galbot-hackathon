# 感知层 SDK 封装指南（B 侧：眼睛 + 大脑）

> 本文对应赛题 **任务二「机器人 SDK 的 Agent 化封装」** 的感知章节。
> 所有 SDK 结论都已对照本仓库内的 `GalbotSDK`（V1.10.0，`linux-x86_64-gcc940`）
> 与官方 Python 示例核实，行号可复查。
> 本文 + `robot_tools/perception.py` + `algorithms/` + `agent/` 构成 B 侧完整交付。
>
> ✅ **已在本机跑通**：合成场景端到端（与真机完全相同的解码路径），检测值与真值一致。
> ❌ **未在真机验证**：见 §7 的诚实边界清单。

---

## 1. 审查结论：Cline 生成的 `perception.py` 能不能用？

**结论：思路对，但直接接官方 SDK 会全面失败。** 3 个必炸的 bug + 4 个结构性问题。

它的骨架是对的（Adapter 模式、把图像标准化、处理 depth_scale），
但**它假设的 SDK 返回形态和真实 SDK 不一致**，而且它没实现协议里真正要交付的 Tool。

### 1.1 必炸的三个 bug

| # | Cline 的代码 | 官方真实行为 | 后果 |
|---|---|---|---|
| **B1** | `depth_raw, depth_scale = result[0], result[1]`（第 71–72 行）<br>把返回值当**二元组** | `get_depth_data()` 返回**单个 dict**：<br>`{'header','format','depth_scale','height','width','data'}`<br>（`g1/__init__.pyi:2391-2414`） | `result[0]` 直接 `KeyError`/`TypeError`，被 `except Exception: return None` 吞掉 → **永远拿到 None**，深度功能整体静默失效 |
| **B2** | `depth_raw / depth_scale`（第 78 行）<br>假设 `data` 已是 numpy 数组 | `data` 是**编码后的字节**，必须<br>① `np.frombuffer(data, uint16).reshape(h,w)`（裸 16UC1）<br>② 或 `cv2.imdecode(..., IMREAD_UNCHANGED)`（compressedDepth PNG）<br>（`examples/g1/python/galbot_robot/get_camera_data.py:72-86`） | 假数据能过，**真机必然报错**。这是"Mock 通过、真机炸"的典型样本 |
| **B3** | `camera_name="HEAD_LEFT_CAMERA"` 传**字符串** | SDK 要 `SensorType` **枚举**；<br>`SensorType.__init__(value: SupportsInt)`，<br>`SensorType("HEAD_LEFT_CAMERA")` → TypeError<br>（`g1/__init__.pyi:5184`） | 真机上每个相机调用都抛异常 → 全被吞成 None |

### 1.2 四个结构性问题（不是"能跑就不管"，而是接不上协议）

| # | 问题 | 为什么在比赛里致命 |
|---|---|---|
| **S1** | 异常一律 `except Exception: return None` | 协议 §1 明确要求 `status` + `error_code`。全部吞成 None 后，Agent 无法区分"相机没装配"和"网络断了"，只能瞎猜并编造答案 |
| **S2** | 没有 `capture_scene()` | 协议 §3 的 6 个 Tool 里，`capture_scene(camera_id)` 是 **B 唯一的强制 Tool**。Cline 只写了内部 helper，`app.py` 没东西可注册 |
| **S3** | RGB 默认 `HEAD_LEFT_CAMERA`，深度默认 `LEFT_ARM_DEPTH_CAMERA` | **两个不同物理相机**，像素不对应。`pixel_to_3d` 用 A 相机的内参去解释 B 相机的画面，3D 坐标会错几十厘米 |
| **S4** | `adapter.get_rgb_data()` / `get_depth_data()` 只存在于 Cline 的想象里 | 仓库里没有 `adapters/`。这个文件当前**根本无法运行** |

### 1.3 它做对的部分（保留）

- ✅ 用 Adapter 隔离真机/Mock——这正是协议 §4 的分层，方向完全正确。
- ✅ `depth_m = depth_raw / depth_scale`（**除法**）——单位判断正确，官方口径一致（`g1/__init__.pyi:2417-2419`）。
- ✅ `depth_m == 0 → NaN`——无效值处理正确。
- ✅ RGB 走 `cv2.imdecode`——对 JPEG 路径正确。

---

## 2. 已核实的 SDK 相机 API（真机对接的唯一依据）

来源：`GalbotSDK/galbot_sdk/linux-x86_64-gcc940/lib/python/galbot_sdk/g1/__init__.pyi`

### 2.1 初始化

```python
from galbot_sdk.g1 import GalbotRobot, RgbOutputFormat, SensorType

robot = GalbotRobot()
ok = robot.init(enable_sensor_set={SensorType.LEFT_ARM_CAMERA,
                                   SensorType.LEFT_ARM_DEPTH_CAMERA},
                enable_sync_mode=False)      # pyi:2772-2790
if not ok:
    raise RuntimeError("init 失败")
```

- `init()` **只生效一次**；重复调用不报错但没用（`pyi:2781-2783`）。
- 只把**真正要用**的传感器放进 `enable_sensor_set`，否则开销白付。
- `get_synced_observation()` 必须 `enable_sync_mode=True` 才能用（`pyi:2708`）。

### 2.2 退出（顺序固定，否则资源泄漏）

```python
robot.request_shutdown()   # pyi:2839，异步，不阻塞
robot.wait_for_shutdown()  # pyi:3351
robot.destroy()            # pyi:2259，之后进程内不可再 init（pyi:2264-2269）
```

### 2.3 彩色图

```python
rgb = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA,
                         RgbOutputFormat.JPEG,   # 默认值就是 JPEG
                         True)                   # once=True：阻塞等一帧新图
```

签名：`get_rgb_data(camera_id: SensorType, format: RgbOutputFormat = JPEG, once: bool = True) -> dict`（`pyi:2650`）

返回 dict 的键（`pyi:2664-2671`）：

| 键 | 说明 |
|---|---|
| `header` | `Header{timestamp_ns, frame_id}`；**DMA-FD 路径的 `frame_id` 是空字符串**（`pyi:2666`） |
| `format` / `output_format` | 字符串与 `RgbOutputFormat` 两种表示 |
| `width` / `height` | 像素尺寸（**由 config 决定，不是常量**） |
| `plane_count` / `stride_bytes` / `plane_offset_bytes` | 平面与行跨距 |
| `data` | JPEG 字节；或紧密排列的 NV12/BGR/RGB 字节 |

`RgbOutputFormat`：`JPEG(0) / NV12(1) / BGR(2) / RGB(3)`（`pyi:4840-4859`）

> **选型建议**：网络/序列化场景用 `JPEG`（硬件编码、体积小、无 stride 问题）。
> 要零解码开销就 `BGR`（紧密排列、直接就是 OpenCV 布局）——**但必须按 `stride_bytes` 解析**。
> 本仓库两种都实现了。

### 2.4 深度图

```python
depth = robot.get_depth_data(SensorType.LEFT_ARM_DEPTH_CAMERA)
```

签名：`get_depth_data(camera_id: SensorType) -> dict`（`pyi:2391`）

| 约束 | 内容 |
|---|---|
| 合法相机 | **只接受** `LEFT_ARM_DEPTH_CAMERA` / `RIGHT_ARM_DEPTH_CAMERA`（`pyi:2395-2396`） |
| 机型 | 仅 G1 / S1；**G3 没有臂上深度**（`pyi:2398-2399`） |
| 返回键 | `header` / `format` / `depth_scale` / `height` / `width` / `data`（`pyi:2405-2411`） |
| 数据类型 | 解码后是 **`uint16` 单通道**，不是 float |
| 单位公式 | `depth_m = pixel_value / depth_scale`（`pyi:2416-2419`） |
| `depth_scale=1000` | 1000 counts = 1.0 m（即原始单位 mm） |
| 无效值 | 像素值 `0` 表示无效/超量程（`pyi:2420`） |

> ⚠️ **两个真实存在的格式陷阱**（本仓库都处理了）：
> 1. `format` 可能是 `"16UC1"`，也可能长成 `"16UC1; compressedDepth png"`。
>    官方示例 `get_camera_data.py:37` 用 `format == "16UC1"` **精确匹配**，遇到后者会
>    抛 `Unsupport data format`。**不要用 format 字符串当唯一判据。**
> 2. 因此本仓库用**字节长度**做主判据：`len(data) == h*w*2` → 裸 uint16 直读；
>    否则 → `cv2.imdecode(IMREAD_UNCHANGED)`，还不行再跳过 12 字节 `cv::Mat` 头重试。

### 2.5 内参

```python
info = robot.get_camera_intrinsic(SensorType.LEFT_ARM_CAMERA)
```

签名：`get_camera_intrinsic(camera_id: SensorType) -> dict`（`pyi:2354`）

| 键 | 内容 |
|---|---|
| `header` | 时间戳 + **相机 frame_id**（RGB 的 header 没 frame_id，要从这里拿） |
| `width` / `height` | **标定时的**分辨率，可能与当前输出不同 |
| `K` | 3×3 行主序：`K[0]=fx, K[4]=fy, K[2]=cx, K[5]=cy`（`pyi:2367-2368`） |
| `D` | 畸变系数，`plumb_bob` 为 `[k1,k2,p1,p2,k3]` |
| `distortion_model` | `plumb_bob` / `rational_polynomial` / `equidistant` |
| `camera_type` | 非 ROS2 标准扩展字段（可用来认出 D405 / RealSense） |

> ⚠️ **命名陷阱**：C++ 头文件用小写 `k`/`d`/`r`/`p`，
> **Python dict 用大写 `K`/`D`**。本仓库的解析器两种都吃，但你写 C++ 时别混。

### 2.6 外参

```python
tf, ts = robot.get_sensor_extrinsic(SensorType.LEFT_ARM_DEPTH_CAMERA, "base_link")
```

签名：`get_sensor_extrinsic(sensor_id, reference_frame="base_link") -> (list[7], int)`（`pyi:2673`）

- 返回 `[x, y, z, qx, qy, qz, qw]`，平移 **米**，四元数 **scalar-last**。
- 失败返回空 list + 时间戳 0 → **必须判断是否为空**，不能当成单位矩阵蒙混。

### 2.7 可用机位（`SensorType`，`pyi:5100-5176`）

| 枚举 | 值 | 含义 | 深度 |
|---|---|---|---|
| `HEAD_LEFT_CAMERA` | 0 | 头左（广角） | ✗ |
| `HEAD_RIGHT_CAMERA` | 1 | 头右（构成双目） | ✗ |
| `LEFT_ARM_CAMERA` | 2 | 左臂彩色 | ✗ |
| `RIGHT_ARM_CAMERA` | 3 | 右臂彩色 | ✗ |
| `LEFT_ARM_DOWN_CAMERA` | 4 | 左臂下视（G3） | ✗ |
| `RIGHT_ARM_DOWN_CAMERA` | 5 | 右臂下视（G3） | ✗ |
| `LEFT_ARM_DEPTH_CAMERA` | 6 | 左臂深度 | ✓ |
| `RIGHT_ARM_DEPTH_CAMERA` | 7 | 右臂深度 | ✓ |
| `LEFT_ARM_INFRA_CAMERA_1/2` | 8/9 | 左臂红外 | ✗ |
| `RIGHT_ARM_INFRA_CAMERA_1/2` | 10/11 | 右臂红外 | ✗ |
| `LEFT/RIGHT_FRONT_SURROUND_CAMERA` | 22/23 | 前环视 | ✗ |
| `LEFT/RIGHT_REAR_SURROUND_CAMERA` | 24/25 | 后环视 | ✗ |

> ⚠️ **SDK 没有"本机装了哪些相机"的查询接口**（`get_device_information` 只给型号/序列号）。
> 机位**登记 ≠ 已装配**；未装配的机位会返回空 dict，本仓库翻译成 `not_ready`，不伪造成功。
> 想知道 TF 里有哪些 frame：`robot.get_frame_names()`（`pyi:2483`）。

### 2.8 同步观测（一次拿"图 + 关节"，给 VLA / 模仿学习用）

```python
robot.init(sensor_set, enable_sync_mode=True)          # 必须
obs = robot.get_synced_observation([SensorType.LEFT_ARM_CAMERA], with_joint_state=True)
```

- `cameras[0]` 是 **anchor**，其余相机按时间戳**最近邻**匹配（软件对齐，非硬件同步）。
- **不设最大时间差上限**：时间戳跳变不会报错，严格同步要自己校验 delta。
- 同步模式下 RGB 是 **NV12**，不是 JPEG（`pyi:2705-2706`）。

---

## 3. A 需要实现的 Adapter 契约（两人对齐的唯一接口）

感知层**不 import `galbot_sdk`**，也不 import `adapters/`。
它只要求 adapter 提供 4 个方法 + 1 个属性，用 `typing.Protocol` 写在
`robot_tools/perception.py::CameraAdapter`。

```python
class CameraAdapter(typing.Protocol):
    mode: str                                                    # "mock" | "real"

    def get_rgb_data(self, sensor: str) -> typing.Any: ...
    def get_depth_data(self, sensor: str) -> typing.Any: ...
    def get_camera_intrinsic(self, sensor: str) -> typing.Any: ...
    def get_sensor_extrinsic(self, sensor: str,
                             reference_frame: str = "base_link") -> typing.Any: ...
```

### 3.1 三条硬约定

1. **`sensor` 是 `SensorType` 的成员名字符串**（如 `"LEFT_ARM_CAMERA"`）。
   **真机适配器内部**做名字 → 枚举的转换：
   ```python
   from galbot_sdk.g1 import SensorType
   cam = SensorType.__members__[sensor]     # ✅
   # cam = SensorType(sensor)               # ❌ 只吃 int，会 TypeError
   ```
   这样感知层可以完全脱离 `galbot_sdk`，在 Windows 上单测。

2. **`mode` 属性必须存在且只能是 `"mock"` 或 `"real"`**。
   协议 §1 禁止静默回退——真机断了不许自己改回 mock。本仓库 `PerceptionTool`
   在构造时就校验，非法值直接 `ValueError`。

3. **返回 SDK 原始 dict 即可**，不用自己解码。
   感知层的 `normalize_payload()` 同时兼容：
   - SDK 原生 dict（`header` 是 pybind 对象、`format` 是枚举）；
   - 已归一化的 dict（`header` 是普通 dict、`format` 是字符串）。
   **取不到数据就返回 `None` / `{}`** —— 感知层会翻译成 `not_ready`。
   **绝对不要**返回一个全零数组假装有数据。

### 3.2 真机适配器骨架（A 可以直接抄）

```python
# adapters/galbot_adapter.py
from __future__ import annotations
from galbot_sdk.g1 import GalbotRobot, RgbOutputFormat, SensorType


class GalbotAdapter:
    """真机适配器：SDK -> 感知层。只做透传与名字解析，不做解码。"""

    mode = "real"

    def __init__(self, sensors: set[SensorType] | None = None) -> None:
        self._robot = GalbotRobot()
        enable = sensors or {
            SensorType.LEFT_ARM_CAMERA,
            SensorType.LEFT_ARM_DEPTH_CAMERA,
        }
        if not self._robot.init(enable):
            raise RuntimeError("GalbotRobot.init() 返回 False")

    # ---- 名字 -> 枚举（唯一一处 SDK 细节） ----
    @staticmethod
    def _cam(sensor: str) -> SensorType:
        try:
            return SensorType.__members__[sensor]
        except KeyError:
            raise ValueError(f"未知 SensorType: {sensor!r}") from None

    # ---- 4 个契约方法 ----
    def get_rgb_data(self, sensor: str):
        return self._robot.get_rgb_data(self._cam(sensor), RgbOutputFormat.JPEG, True)

    def get_depth_data(self, sensor: str):
        return self._robot.get_depth_data(self._cam(sensor))

    def get_camera_intrinsic(self, sensor: str):
        return self._robot.get_camera_intrinsic(self._cam(sensor))

    def get_sensor_extrinsic(self, sensor: str, reference_frame: str = "base_link"):
        return self._robot.get_sensor_extrinsic(self._cam(sensor), reference_frame)

    # ---- 退出：顺序固定 ----
    def close(self) -> None:
        self._robot.request_shutdown()
        self._robot.wait_for_shutdown()
        self._robot.destroy()
```

### 3.3 Mock 适配器要注意什么

**Mock 必须输出"编码后的字节"，不能直接吐 numpy 数组。**
否则真机与 Mock 走两条不同的解码路径，"模拟通过、真机炸"就一定会发生。
参考实现见 `agent/offline_demo_adapter.py`：它用 `cv2.imencode('.jpg', ...)` 生成 JPEG、
用 `(depth*1000).astype(uint16).tobytes()` 生成裸 16UC1、`depth_scale=1000`。

> 该文件是 **B 的离线演示适配器**，不是 A 的交付物。A 的 `adapters/mock_adapter.py`
> 还应负责状态机、任务、导航 Mock（协议 §6 要求）。

---

## 4. 交给大模型的 Tool 长什么样

协议 §3 规定的 6 个 Tool 中，B 负责 `capture_scene`。本仓库额外补了 4 个
**面向"眼睛"能力**、但不越权的 Tool（都只读，不驱动硬件）：

| Tool | 输入 | 给模型的关键返回 |
|---|---|---|
| `list_cameras()` | — | 已登记机位、是否有深度、适用距离 |
| `capture_scene(camera_id, include_depth)` | 白名单机位 | `capture_id` / `rgb_ref` / `depth_ref` / `captured_at` / `coordinate_frame` / `depth_unit="m"` |
| `analyze_scene(camera_id, want_labels, max_objects)` | 可选颜色过滤 | `objects[]`：label、bbox、中心像素、`depth_m`、`center_cam_m`、`fusion` 状态 |
| `find_object(label, camera_id, max_distance_m, with_base_link)` | 颜色或 `foreground` | 单目标 + `base_link_pose_m` |
| `measure_distance(u, v, camera_id)` | 像素坐标 | 该点距离（邻域中位数，抗噪） |

**设计要点（都是给模型省 token、给评委省事故）：**

1. **返回引用 ID，不返回像素**。`rgb_ref = "cap_...:rgb"`，`FrameStore` 用严格正则
   校验，`../`、绝对路径、盘符全部拒绝。
2. **每个 3D 点都带坐标系**，`coordinate_frame` 取内参 header 的 frame_id；
   拿不到就显式写成 `unverified:<sensor>_optical_frame` 并给 warning，不硬编造。
3. **深度融合失败要说原因**，`fusion` 字段有 4 种取值：
   `camera_frame`（成功）/ `no_depth_sensor` / `shape_mismatch:...` / `no_valid_depth_in_mask`。
4. **尺寸不一致不猜**：彩色与深度分辨率不同时记 warning 且不做融合——
   官方**没有**给出两者像素对齐的保证（这正是初版 S3 踩的坑）。
5. **全部 9 个 error_code 都用得上**：非法机位 → `invalid_argument`；
   无深度能力的机位 → `not_supported`；相机未 enable → `not_ready`；
   解码失败 → `sdk_error`。

---

## 5. 模块清单与调用链

```
agent/app.py                  CLI + Agent 主循环（LLM -> tool -> result）
  └── agent/tool_registry.py  schema 生成 / 参数前置校验 / 分发
  └── agent/llm_client.py     OpenAI 兼容客户端 + 离线规则引擎
  └── agent/offline_demo_adapter.py  合成场景（离线演示用，非 A 的交付物）

robot_tools/perception.py     ★ 核心：payload 归一化、解码、反投影、5 个 Tool
  └── algorithms/scene_analyzer.py   2D 检测 + 深度融合 -> 结构化场景
        └── algorithms/color_detection.py  HSV 颜色检测（可替换为 YOLO）
        └── algorithms/geometry.py         四元数/外参/反投影/掩膜统计
  └── algorithms/place_resolver.py  自然语言地名 -> place_id 白名单

contracts.py                  冻结契约：ToolResult / 6 status / 9 error_code
frame_store.py                帧缓存 + 不可伪造的引用 ID
config/cameras.json           相机白名单（别名、SensorType、是否有深度）
```

**依赖方向铁律**：`agent → robot_tools → algorithms`，**绝不反向**，
且 `robot_tools/perception.py` 与 `algorithms/*` 都不 import `galbot_sdk`。
这就是它能在一台没装 SDK 的机器上跑通 `--demo` 的原因。

---

## 6. 怎么跑

### 6.1 本机（Windows / 无机器人 / 无 SDK）

```powershell
cd D:\Year2_BUAA\Junction\galbot_dev
pip install -r requirements.txt

python -m agent.app --list-tools     # 看注册了哪些工具
python -m agent.app --demo           # 跑完整演示（离线规则引擎，无需网络）
python -m agent.app --once "红色方块在哪，离我多远"
python -m agent.app                  # 交互式 REPL
```

接真模型（任何 OpenAI 兼容端点，改 `--base-url` 即可）：

```powershell
$env:OPENAI_API_KEY = "sk-xxx"
python -m agent.app --model qwen-vl-max `
    --base-url https://dashscope.aliyuncs.com/compatible-mode/v1 --vision
```

> `--vision` 会把抓到的帧缩到 768px、base64 塞进 `image_url`。
> 不开也能用——结构化场景描述本身就是文本，这是刻意的降级设计。

### 6.2 真机（VMware Ubuntu 22.04 + G1）

```bash
cd ~/galbot_dev
export LD_LIBRARY_PATH=/opt/galbot/galbot_sdk/linux-x86_64-gcc940/lib
export PYTHONPATH=/opt/galbot/galbot_sdk/linux-x86_64-gcc940/lib/python:$PYTHONPATH

python -m agent.app --adapter real --once "你看到了什么"
```

**前提**：A 已交付 `adapters/galbot_adapter.py`。若缺失，程序会
**明确报错退出（exit 2）**，而不是偷偷退回 mock。

### 6.3 上真机前的 6 步逐项核对

1. `robot.init(...)` 返回 True。
2. `get_rgb_data(...)` 的 `format` / `width×height` / `bytes` 打印出来。
3. `get_depth_data(...)` 的 `format` / `depth_scale` / `width×height` 打印出来，
   **并与 RGB 的尺寸比对**，决定能否融合。
4. `depth_m` 是 `pixel / depth_scale`，拿卷尺量 0.5 m 处验证。
5. `get_camera_intrinsic(...)` 的 `K` / `D` / `width` / `height` 打印，
   和 `config/cameras.json` 里的分辨率约定对照。
6. `get_sensor_extrinsic(...)` 返回非空，且四元数是 **xyzw** 顺序。

---

## 7. 诚实边界（未验证 / 不确定的事）

比赛文档里最忌讳的是"没跑通却写成支持"。以下事项**尚未验证**：

| 事项 | 状态 | 影响 |
|---|---|---|
| 真机上的真实 `format` 字符串 | **未验证** | 已做长度判据 + PNG 双路径兜底，但仍需真机打印确认 |
| 深度与彩色的像素对齐 | **官方无保证** | 代码里不猜：尺寸不一致就拒融合并告警 |
| 各机位在本机的实际装配情况 | **无法查询** | 未装配 → `not_ready`，不伪造 |
| `get_sensor_extrinsic` 的可用性 | **未验证** | 拿不到就 `base_link_pose_m=null` + warning |
| 头部双目深度图 | **已实现独立 SDK 工具** | `StereoDepthTool.capture_stereo_depth` 调用 `FOUNDATION_STEREO`；返回原始 uint 数值并标记 `unit="mm", raw_to_m=0.001`。2026-11-08 已用头左外参和地面平面验证尺度；反投影前需按 `kannala_brandt` 鱼眼模型去畸变 |
| 语义检测（非颜色） | **未实现** | `algorithms/color_detection.py` 是替换点；接 YOLO/GroundingDINO 只改 `detect()` |
| 与 A 的 `adapters/` 联调 | **未联调** | 契约已冻结（§3），A 按契约实现即可插上 |
| G3 / S1 机型 | **未覆盖** | G3 无臂上深度相机，本仓库按 G1 设计 |

**下一步（按优先级）**：

1. A 交付 `adapters/galbot_adapter.py`（§3.2 骨架可直接抄）+ `mock_adapter.py`。
2. 真机跑 §6.3 的 6 步核对，把打印结果回填到本文 §2。
3. 用 `example4_sensor_data_collect.py` 的采集逻辑对齐"图 + 关节"时间戳。
4. 需要语义检测时替换 `color_detection.detect()`，接口不变。

---

## 8. 与赛题交付物的关系

赛题要求的是一份 `.md`：「把 SDK 打包成一个个 tool，可供多模态模型自主调用；
先抓一遍 SDK 并生成一份能直接开发的经验/解释文档」。

本文就是**感知章节**的成稿，具备可直接开发的三要素：

- **签名级事实**（§2，全部带 `.pyi` 行号，可复查）；
- **两人协作契约**（§3，A 按此实现即可插上，含可抄骨架）；
- **踩坑清单**（§1 的三个必炸 bug、§2.4 的格式陷阱、§7 的未验证清单）。

建议与 A 的运动/导航章节合并为 `docs/sdk_agent_guide.md`，共用同一套
`ToolResult` 契约与同一张"已验证 / 未验证"表——**统一的风险披露表**
往往是这类赛题评分里最容易拿分、也最容易被别人忽略的一项。

