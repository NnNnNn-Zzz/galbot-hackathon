#!/bin/bash
# G1 真机模式环境变量。用法：source scripts/env.sh
#
# 解决两件事：
#   1. PYTHONPATH      —— 让 python 找到 galbot_sdk 包
#   2. LD_LIBRARY_PATH —— 让 galbot_sdk 找到 libgalbot_sdk.so 及原生依赖
#
# 前提：SDK 原生依赖（libfastcdr / libfastrtps 等）已安装。
#   没装过的话先执行一次：  sudo ~/GalbotSDK/install.sh
#   装完 ldconfig 会接管，本脚本的 LD_LIBRARY_PATH 只是兜底。

export GALBOT_SDK_ROOT="${GALBOT_SDK_ROOT:-$HOME/GalbotSDK}"
_ARCH=$(uname -m)   # x86_64 或 aarch64

_SDK_LIB=$(ls -d "$GALBOT_SDK_ROOT"/galbot_sdk/linux-${_ARCH}-gcc*/lib 2>/dev/null | head -1)
if [ -z "$_SDK_LIB" ]; then
    echo "[env.sh] 未找到 SDK 库目录，请检查 GALBOT_SDK_ROOT=$GALBOT_SDK_ROOT"
    return 1 2>/dev/null || exit 1
fi

export PYTHONPATH="$_SDK_LIB/python:$PYTHONPATH"
export LD_LIBRARY_PATH="$_SDK_LIB:$LD_LIBRARY_PATH"

echo "[env.sh] GALBOT_SDK_ROOT=$GALBOT_SDK_ROOT"
echo "[env.sh] PYTHONPATH += $_SDK_LIB/python"
echo "[env.sh] LD_LIBRARY_PATH += $_SDK_LIB"
