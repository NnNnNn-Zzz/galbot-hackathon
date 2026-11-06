#!/bin/bash
# 一键部署：WSL → 机器人 HPU。
# 用法（在 WSL 项目根目录）：
#   bash scripts/deploy_hpu.sh [HPU_IP]
# 默认 HPU_IP=192.168.166.7（场地 WiFi DHCP 分配，IP 变了就传参覆盖）。
#
# 做三件事：
#   1. 推送项目代码（排除 .git / task_log / task.txt / .venv）→ ~/galbot_hackathon
#   2. 推送 aarch64 cp38 Python SDK 包 → ~/g1_runtime（幂等，已存在则增量跳过）
#   3. HPU 端跑一次 mock 冒烟自检（不动真机）
#
# SSH 密码 galbot 默认官方密码，rsync/ssh 会提示输入；需要免密可自行配 ssh key。

set -e
HPU_IP="${1:-192.168.166.7}"
SDK_SRC="$HOME/GalbotSDK/galbot_sdk/linux-aarch64-gcc940/lib/python/galbot_sdk"

echo "== [1/3] 推送项目代码 → galbot@$HPU_IP:~/galbot_hackathon =="
ssh -o StrictHostKeyChecking=no galbot@"$HPU_IP" "mkdir -p ~/galbot_hackathon ~/g1_runtime"
rsync -av --delete \
      --exclude '.git' --exclude '.venv' --exclude 'task_log.jsonl' \
      --exclude 'task.txt' --exclude '__pycache__' --exclude '*.pyc' \
      ./ "galbot@$HPU_IP:~/galbot_hackathon/"

echo "== [2/3] 推送 aarch64 Python SDK → ~/g1_runtime =="
rsync -av --exclude 'galbot_sdk.cpython-39-*' \
      --exclude 'galbot_sdk.cpython-310-*' --exclude 'galbot_sdk.cpython-311-*' \
      --exclude 'galbot_sdk.cpython-312-*' --exclude 'galbot_sdk.cpython-313-*' \
      --exclude 'galbot_sdk.cpython-314-*' --exclude '__pycache__' \
      "$SDK_SRC/" "galbot@$HPU_IP:~/g1_runtime/galbot_sdk/"

echo "== [3/3] HPU 端 mock 冒烟自检 =="
ssh -o StrictHostKeyChecking=no galbot@"$HPU_IP" \
    "cd ~/galbot_hackathon && source scripts/env_hpu.sh && \
     python3 scripts/test_base_motion.py --mock --distance 10 2>&1 | tail -5"

echo "== 部署完成。SSH 上去后：source scripts/env_hpu.sh && python3 agent/main.py --task \"往前走50cm\" =="
