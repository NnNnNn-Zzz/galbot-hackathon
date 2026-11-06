"""task_log 会话记忆：JSONL 追加写，Agent 每轮决策和执行结果都留痕。"""

import json
import os
import time

_LOG_PATH = os.path.join(os.path.dirname(__file__), "..", "task_log.jsonl")


def log(event_type: str, content: dict):
    """追加一条记录。event_type: decision / scene_result / wrapper_call / unsafe_exit"""
    entry = {"ts": time.strftime("%Y-%m-%d %H:%M:%S"), "type": event_type, **content}
    with open(_LOG_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def read_history(n: int = 10) -> list:
    """读最近 n 条，转换为 LLM 消息格式。"""
    if not os.path.exists(_LOG_PATH):
        return []
    with open(_LOG_PATH, "r", encoding="utf-8") as f:
        lines = f.readlines()[-n:]
    history = []
    for line in lines:
        try:
            entry = json.loads(line)
            history.append({"role": "user",
                            "content": f"[{entry['ts']}] {entry['type']}: "
                                       + json.dumps(entry, ensure_ascii=False)})
        except json.JSONDecodeError:
            continue
    return history
