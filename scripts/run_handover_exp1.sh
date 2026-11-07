#!/bin/bash
# 交接即用：实验 1（空桌盲抓，复现 16:28 那次 FAULT 条件）一键执行。
#
# 用法（HPU 上，交接拿到机器人之后）：
#     cd ~/galbot_hackathon && bash scripts/run_handover_exp1.sh
#
# 它做四件事，全程 <11 分钟：
#   [0:00] 只读读取当前位置 —— 记录接手时姿态（零运动，不做任何纠正）
#   [0:30] 只读预检（配置/环境自检）—— 不过就立刻停，不冒险
#   [1:30] 空桌确认提示 —— 必须你亲手按回车
#   [2:00] 实验 1 正式执行，日志双写到 handover_exp1_<时间>.log
#
# 安全约定：
#   * 前两步只有读接口，绝无运动
#   * 本脚本不会跳过确认，不提供 --yes
#   * 看到 unknown 时实验会自己全停；你只需要拍急停或让开
#   * 脚本本身不含安全闸的绕过逻辑（实验 3 才有闸门）

set -u

TS="$(date +%H%M%S)"
LOG="handover_exp1_${TS}.log"

echo "================================================================"
echo " 交接实验 1 · 空桌盲抓 · 日志 $LOG"
echo "================================================================"

if [ ! -f scripts/env_hpu.sh ]; then
    echo "[ABORT] 当前目录不是 ~/galbot_hackathon（找不到 scripts/env_hpu.sh）"
    echo "        先 cd ~/galbot_hackathon 再来"
    exit 2
fi

# env_hpu.sh 里用的是 $PYTHONPATH 展开，在 set -u 下未定义会报错。
# 先给个空值兜底，再 source（不影响它自身的追加逻辑）。
: "${PYTHONPATH:=}"
: "${LD_LIBRARY_PATH:=}"
# shellcheck disable=SC1091
source scripts/env_hpu.sh

echo
echo "--- [0/3] 只读读取当前位置（**零运动**：不写关节、不动夹爪、不动底盘）---"
echo "    刚才有人拖过手臂 —— 这一步只记录接手时的真实姿态，不做任何纠正"
echo
python3 scripts/read_current_pose.py 2>&1 | tee -a "$LOG"
READ_RC="${PIPESTATUS[0]}"
if [ "$READ_RC" -ne 0 ]; then
    echo
    echo "[ABORT] 连位置都读不到或读数不可信（exit=$READ_RC）。"
    echo "        不要动臂。先按上面提示查 DDS / robot_server / env 是否 source。"
    exit 2
fi

echo
echo "--- [1/3] 只读预检（配置/环境自检）---"
python3 scripts/preflight_readonly.py 2>&1 | tee -a "$LOG"
PRE="${PIPESTATUS[0]}"
if [ "$PRE" -ne 0 ]; then
    echo
    echo "[ABORT] 只读预检未通过（exit=$PRE）。按上面 blockers 处理，不要动臂。"
    exit 2
fi

echo
echo "--- [2/3] 空桌确认 ---"
echo "  请现在亲手确认（不要凭记忆）："
echo "    [ ] 桌面完全清空：没有盒子、没有纸片、没有卷尺、没有胶带卷"
echo "    [ ] 机器人前方 1 米内没有人站着"
echo "    [ ] 急停在够得着的地方"
echo "    [ ] 你清楚接下来手臂会往 (0.40, -0.10, 0.77) 伸，然后抬起收臂"
echo
read -r -p "  全部确认后按回车开始（Ctrl+C 取消）：" _

echo
echo "--- [3/3] 执行实验 1 —— 盯住手臂，别盯终端 ---"
START=$(date +%s)
MED_IO=wrapper python3 scripts/run_exp1_grasp_only.py 2>&1 | tee -a "$LOG"
RC="${PIPESTATUS[0]}"
END=$(date +%s)
ELAPSED=$((END - START))

echo
echo "================================================================"
echo " 结束 · 退出码 $RC · 用时 ${ELAPSED}s · 日志 $LOG"
case "$RC" in
    0) echo " 判定：通过。三步全 ok。下一步可以谈实验 2。" ;;
    1) echo " 判定：不通过。看日志最后的 verdict.reasons，不要盲目重跑。" ;;
    2) echo " 判定：配置/前置检查未过，一个动作都没发。" ;;
    *) echo " 判定：异常退出码 $RC，先看日志尾部。" ;;
esac
echo "================================================================"
exit "$RC"
