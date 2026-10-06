#!/usr/bin/env bash
# direct_thruster M4 — 一键跑官方 SITL 验收 (在 WSL 里运行, 不碰真机)。
#
#   bash /mnt/c/bluerov2_mpc/direct_thruster/sitl_run.sh              # 起 SITL 跑验收再收尾
#   bash /mnt/c/bluerov2_mpc/direct_thruster/sitl_run.sh --keep       # 跑完留着 SITL
#   bash /mnt/c/bluerov2_mpc/direct_thruster/sitl_run.sh --sitl-only  # 只起 SITL, 不跑验收
#   bash .../sitl_run.sh --run "python3 .../external_thruster.py --zero --seconds 5"
#                                                                 # 起 SITL 跑一条命令再收尾
#
# 后两个用来验上位机 external_thruster.py —— 它需要一个还活着的 SITL。
#
# 为什么要有这个脚本, 而不是手敲两条命令:
#   1) SITL 必须用 FRAME_CONFIG=2 起 (见 sitl_ext.parm)。手敲时漏掉这一条,
#      Motor7/8 不被使能、SERVO7/8 恒为 0, 所有"全中位"判据会整体报红
#      —— 首轮 0/11 就是这么来的。把它写死在脚本里就漏不掉。
#   2) SITL 与验收脚本必须在同一个 shell 会话里; 会话一结束 SITL 就被杀。
#   3) 必须 -w 擦掉上次残留的 eeprom.bin, 否则旧参数会盖掉 defaults 文件。
set -u

KEEP=0
SITL_ONLY=0
RUN_CMD=""
while [ $# -gt 0 ]; do
    case "$1" in
        --keep)      KEEP=1 ;;
        --sitl-only) SITL_ONLY=1; KEEP=1 ;;
        --run)       shift; RUN_CMD="${1:-}"
                     [ -n "$RUN_CMD" ] || { echo "--run 后面要跟命令" >&2; exit 2; } ;;
        -h|--help)   sed -n '2,16p' "$0"; exit 0 ;;
        *) echo "未知参数: $1 (可用: --keep / --sitl-only / --run <命令>)" >&2; exit 2 ;;
    esac
    shift
done

SRC="${SRC:-$HOME/rov-dev/ardupilot-external}"
REPO="${REPO:-/mnt/c/bluerov2_mpc}"
RUNDIR="${RUNDIR:-/tmp/sitlrun}"

BIN="$SRC/build/sitl/bin/ardusub"
BASE_PARM="$SRC/Tools/autotest/default_params/sub.parm"
EXT_PARM="$REPO/direct_thruster/sitl_ext.parm"
ACCEPT="$REPO/direct_thruster/sitl_accept.py"

for f in "$BIN" "$BASE_PARM" "$EXT_PARM" "$ACCEPT"; do
    if [ ! -f "$f" ]; then
        echo "[run] 缺少文件: $f" >&2
        [ "$f" = "$BIN" ] && echo "[run] 先编 SITL: cd $SRC && python3 ./waf configure --board sitl && python3 ./waf sub" >&2
        exit 2
    fi
done

# 清掉上一轮残留的 SITL (模式写得很窄, 只打自己这份构建产物)
if pgrep -f "build/sitl/bin/ardusub" >/dev/null 2>&1; then
    echo "[run] 发现残留 SITL 进程, 杀掉"
    pkill -f "build/sitl/bin/ardusub"
    sleep 1
fi

mkdir -p "$RUNDIR"
cd "$RUNDIR" || exit 2
rm -f eeprom.bin dataflash.bin sitl.log accept.log

# 把 parm 和验收脚本拷进 WSL 自己的文件系统再用。跑的过程中完全不碰 /mnt/c:
# 9p 的访问会被 Windows 侧(杀毒实时扫描等)拖住, 而本验收全靠计时, 进程被冻结
# 数秒就会让 SITL 的 millis() 跳过 MOT_EXT_TMOUT, 看门狗正确锁存 -> 一堆假失败。
cp -f "$EXT_PARM" "$RUNDIR/sitl_ext.parm" || exit 2
cp -f "$ACCEPT"   "$RUNDIR/sitl_accept.py" || exit 2
EXT_PARM="$RUNDIR/sitl_ext.parm"
ACCEPT="$RUNDIR/sitl_accept.py"

echo "[run] 启动 SITL (FRAME_CONFIG=2, 擦 eeprom) ..."
setsid "$BIN" -w -S -I0 --model vectored_6dof \
    --defaults "$BASE_PARM,$EXT_PARM" \
    > sitl.log 2>&1 < /dev/null &
SITL_PID=$!
cleanup() {
    if [ "$KEEP" -eq 1 ]; then
        return
    fi
    if kill -0 "$SITL_PID" 2>/dev/null; then
        kill "$SITL_PID" 2>/dev/null
        sleep 0.5
        kill -9 "$SITL_PID" 2>/dev/null
    fi
}
trap cleanup EXIT INT TERM

# 等 MAVLink TCP 端口就绪 (不去 connect 探测: SITL 的 5760 一次只收一个客户端,
# 探测连接会和验收脚本抢这个名额)
ready=0
for _ in $(seq 1 60); do
    if grep -qE "TCP port 5760|Waiting for connection" sitl.log 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SITL_PID" 2>/dev/null; then
        echo "[run] SITL 启动即退出, sitl.log 尾部:" >&2
        tail -n 30 sitl.log >&2
        exit 2
    fi
    sleep 0.5
done
if [ "$ready" -ne 1 ]; then
    echo "[run] 30s 内没等到 SITL 的 TCP 端口, sitl.log 尾部:" >&2
    tail -n 30 sitl.log >&2
    exit 2
fi
if grep -qiE "invalid param|failed to load defaults" sitl.log; then
    echo "[run] ⚠ defaults 文件有问题:" >&2
    grep -iE "invalid param|failed to load defaults" sitl.log >&2
fi
if [ "$SITL_ONLY" -eq 1 ]; then
    echo "[run] SITL 已就绪并保持运行  pid=$SITL_PID  tcp:127.0.0.1:5760"
    echo "[run] 现在可以跑上位机, 例如:"
    echo "        python3 /mnt/c/bluerov2_mpc/direct_thruster/external_thruster.py --zero --seconds 5"
    echo "        python3 /mnt/c/bluerov2_mpc/direct_thruster/external_thruster.py --arm --sweep --thrust 0.3"
    echo "[run] 用完请收掉:  kill $SITL_PID"
    exit 0
fi

if [ -n "$RUN_CMD" ]; then
    # 起 SITL -> 跑指定命令 -> 收尾。保证同一时刻只有一个客户端连 5760 ——
    # SITL 的 TCP 口一次只接一个, 多连的会被当场关掉, 表现为 pymavlink 刷
    # "EOF on TCP socket"。手动两步时最容易踩到这个。
    echo "[run] SITL 就绪 (pid=$SITL_PID), 执行: $RUN_CMD"
    echo
    sh -c "$RUN_CMD" 2>&1 | tee accept.log
    rc=${PIPESTATUS[0]}
    echo
    echo "[run] 命令退出码 = $rc   (日志: $RUNDIR/accept.log, $RUNDIR/sitl.log)"
    exit "$rc"
fi

echo "[run] SITL 就绪 (pid=$SITL_PID), 开始验收 ..."
echo

timeout 300 python3 -u "$ACCEPT" 2>&1 | tee accept.log
rc=${PIPESTATUS[0]}

echo
echo "[run] 验收脚本退出码 = $rc   (日志: $RUNDIR/accept.log, $RUNDIR/sitl.log)"
if [ "$rc" -ne 0 ]; then
    echo "[run] sitl.log 尾部 20 行:"
    tail -n 20 sitl.log
fi
if [ "$KEEP" -eq 1 ]; then
    echo "[run] --keep: SITL 仍在运行  pid=$SITL_PID  tcp:127.0.0.1:5760   (收掉: kill $SITL_PID)"
fi
exit "$rc"
