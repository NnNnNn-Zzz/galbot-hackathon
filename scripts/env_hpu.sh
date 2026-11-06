#!/bin/bash
# HPU（机器人本体）端环境变量。在机器人上跑 agent/测试前先 source 本文件。
#
# 前提（部署脚本 deploy_hpu.sh 已自动完成）：
#   - ~/g1_runtime/galbot_sdk  —— 从 PC 推来的 Python SDK 包（cp38）
#   - /data/galbot/lib         —— 机器人自带的 SDK 原生库（版本与固件配套，勿动）
#
# 用法（SSH 到机器人后）：
#   cd ~/galbot_hackathon && source scripts/env_hpu.sh
#   python3 agent/main.py --task "往前走50cm"

export PYTHONPATH="$HOME/g1_runtime:$PYTHONPATH"
export LD_LIBRARY_PATH="/data/galbot/lib:$LD_LIBRARY_PATH"

echo "[env_hpu.sh] PYTHONPATH += ~/g1_runtime"
echo "[env_hpu.sh] LD_LIBRARY_PATH += /data/galbot/lib"
