# 银河通用 G1 黑客松开发框架

40 小时黑客松 · WSL Ubuntu 22.04 · Galbot SDK v1.10.0 · G1 真机

## 架构一句话

```
你的自然语言提示词 → 顶层 Agent（LLM 决策）→ 封装层 SDK（安全限幅）→ G1 真机
                                          ↘ 即插即用场景脚本（注册表调度）
```

**通过改 `agent/prompt.txt` 控制机器人行为**，不改代码。

## 目录结构

```
galbot_hackathon/
├── pyproject.toml              # 统一依赖（仅负责人 A 改）
├── agent/                      # 顶层 Agent（仅负责人 A 改）
│   ├── main.py                 #   主循环：观察 → LLM 决策 → 执行 → 留痕
│   ├── llm_client.py           #   LLM 接口抽象（OpenAI 兼容 API，可换）
│   ├── prompt.txt              #   ★ 你写给机器人的"角色设定"
│   ├── memory.py               #   task_log 会话记忆（JSONL）
│   └── scene_loader.py         #   场景注册表读取 + 动态调用
├── sdk_wrapper/                # 硬件封装层（仅负责人 A 改）
│   ├── wrapper.py              #   move_forward / turn / read_sensor ...
│   ├── safety_config.json      #   ★ 安全限幅参数（人工可调）
│   ├── baseline.py             #   归位函数 + 基线检查
│   └── mock_robot.py           #   无真机时的模拟机器人
├── scenes/                     # 场景脚本（各负责人各管各的文件夹）
│   ├── scene_patrol/           #   示例：巡逻
│   └── scene_greet/            #   示例：打招呼
├── registry/
│   └── scenes.yaml             #   ★ 场景统一注册表（追加自己的条目）
└── docs/
    └── DEVELOPMENT_RULES.md    #   ★ 合作开发规范（开工前必读）
```

## 快速开始

```bash
cd ~/galbot_hackathon

# 1. 安装 uv（如果没装）
curl -LsSf https://astral.sh/uv/install.sh | sh

# 2. 同步环境
uv sync

# 3. mock 模式跑通全流程（无真机、无 API Key 也能跑）
uv run python agent/main.py --mock --max-loops 3

# 4. 接入真实 LLM
export LLM_API_KEY="sk-..."
export LLM_BASE_URL="https://api.deepseek.com/v1"   # 任选 OpenAI 兼容 API
export LLM_MODEL="deepseek-chat"
uv run python agent/main.py --mock

# 5. 真机模式（赛时）
uv run python agent/main.py
```

## 顶层 Agent 的 LLM 接口（预留，可换）

`agent/llm_client.py` 定义了 `LLMClient` 基类，只要实现 `decide(system_prompt, observation, history) -> dict` 就能接入任意模型。默认 `OpenAICompatibleClient` 零依赖走 HTTP，兼容 OpenAI / DeepSeek / 通义 / 本地 vLLM，用环境变量切换。未配 Key 时降级为规则模式（只待机，用于调通流程）。Agent 主循环本身也会按比赛实际迭代，接口保持稳定即可。

## 给机器人"立人设"

直接编辑 `agent/prompt.txt`，例如：

> 你是一个家庭安全机器人。每隔一段时间调用 scene_patrol 巡逻；检测到人就调用 scene_greet 打招呼……

Agent 启动时会把 prompt + 封装接口清单 + 场景注册表自动拼成 system prompt。LLM 输出 JSON 决策，主循环执行并把结果写进 `task_log.jsonl`。

## 我要开发一个场景（给场景负责人）

1. 复制 `scenes/scene_greet/` 为 `scenes/scene_你的名字/`。
2. 实现 `run(params) -> dict`，**finally 里必须归位**（`baseline.reset_to_baseline()`）。
3. 硬件调用只用 `sdk_wrapper`；需要新函数/新依赖 → 群里向负责人 A 申请。
4. 在 `registry/scenes.yaml` 追加自己的条目。
5. mock 测试：`GALBOT_MOCK=1 uv run python -c "from scenes.scene_你的名字.scene import run; print(run({}))"`

## 安全机制

- `sdk_wrapper/safety_config.json`：最大速度 0.3 m/s、单次最远 2m、最大转向 180°，超限自动截断。只能人工改（负责人 A），场景运行时改不了。
- 主进程基线服务清单（`galbot_chassis_service` / `singorix` / 急停线程等）任何人不可碰，详见规范文档。
- 场景退出后 Agent 自动做基线检查，异常记 `unsafe_exit` 但不阻塞。

## 分工

| 角色 | 负责 |
|---|---|
| 负责人 A（1 人） | `agent/` + `sdk_wrapper/` + `pyproject.toml` + 真机部署 |
| 场景负责人（N 人） | 各自 `scenes/scene_*/` + `registry/scenes.yaml` 自己的条目 |

详细规则（申请流程、不可碰清单、检查清单）见 **[docs/DEVELOPMENT_RULES.md](docs/DEVELOPMENT_RULES.md)**，开工前全员必读。
