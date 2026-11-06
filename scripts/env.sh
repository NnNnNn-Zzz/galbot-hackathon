#!/bin/bash
# G1 真机模式环境变量。用法：source scripts/env.sh
#
# 解决三件事：
#   1. PYTHONPATH      —— 让 python 找到 galbot_sdk 包
#   2. LD_LIBRARY_PATH —— 让 galbot_sdk 找到 libgalbot_sdk.so 及原生依赖
#   3. 原生依赖来源     —— 优先用离线解压的 ~/.galbot_deps（免 sudo、免网络）
#
# 背景：本机 install.sh 只装了一半（SDK 在 /opt/galbot，第三方 deps 没装），
# 且 WSL 网络不通 PyPI/镜像站。因此 deps 直接从 ~/GalbotSDK/deps/third_party/releases/
# 的本地 tarball 离线解压到 ~/.galbot_deps/lib（一次性，已做过）。

export GALBOT_SDK_ROOT="${GALBOT_SDK_ROOT:-/opt/galbot}"
[ -d "$GALBOT_SDK_ROOT/galbot_sdk" ] || export GALBOT_SDK_ROOT="$HOME/GalbotSDK"
_ARCH=$(uname -m)   # x86_64 或 aarch64

_SDK_LIB=$(ls -d "$GALBOT_SDK_ROOT"/galbot_sdk/linux-${_ARCH}-gcc*/lib 2>/dev/null | head -1)
if [ -z "$_SDK_LIB" ]; then
    echo "[env.sh] 未找到 SDK 库目录，请检查 GALBOT_SDK_ROOT=$GALBOT_SDK_ROOT"
    return 1 2>/dev/null || exit 1
fi

export PYTHONPATH="$_SDK_LIB/python:$PYTHONPATH"
export LD_LIBRARY_PATH="$_SDK_LIB:$LD_LIBRARY_PATH"

# 离线解压的第三方依赖（fastcdr / fastrtps / opencv / pcl 等）
if [ -d "$HOME/.galbot_deps/lib" ]; then
    export LD_LIBRARY_PATH="$HOME/.galbot_deps/lib:$LD_LIBRARY_PATH"
fi

echo "[env.sh] GALBOT_SDK_ROOT=$GALBOT_SDK_ROOT"
echo "[env.sh] PYTHONPATH += $_SDK_LIB/python"
echo "[env.sh] LD_LIBRARY_PATH += $_SDK_LIB, ~/.galbot_deps/lib"
