# direct_thruster — 会话交接文档

> 更新：2026-10-06。新会话请先读本文件，再按需查 [README.md](README.md)（完整方案）、[M0_baseline.md](M0_baseline.md)（实机基线）、[PROGRESS.md](PROGRESS.md)（本线进度流水）。

---

## 1. 核心目标

仓库 `C:\bluerov2_mpc`（GitHub `Yaotseng3tsu/bluerov2_MPC`，main 持续 push）。两条并行线：

- **线 A `waypoint/`**：纯 Python + pymavlink 控制 BlueROV2 R4（**不用 ROS**），走官方 ArduSub 的 `MANUAL_CONTROL`。**已真机下水实测**。
- **线 B `direct_thruster/`（当前焦点）**：**修改并编译 ArduSub C++**，绕过混控分配矩阵，让上位机对 **8 个 T200 逐个连续给归一化推力**（供 MPC / RL / 自定义分配）。MPC、分配器、上位机仍然全部是 Python——只有改机载接口或底层保护才需要重编固件。

---

## 2. 实机基线（M0，已确认并备份）

| 项 | 值 |
|---|---|
| 固件 | **ArduSub 4.1.2**，git `2dd0bb7d`，Board **Navigator**，Vehicle=Submarine |
| BlueOS | **1.4.2**（Bullseye，**32-bit armhf**），有 `RESTORE DEFAULT FIRMWARE` 可回退 |
| 帧 | `FRAME_CONFIG=2` = Vectored_6DOF（**Heavy，8 推进器**） |
| 输出 | **SERVO1–8 = Motor1–8**，未反向，PWM **1100–1900 / TRIM 1500** |
| 参数备份 | `direct_thruster/Submarine-4.1.2-STABLE-20260916-152651.params`（912 行，已入库） |

决策：**版本 A —— 匹配当前 4.1.2 fork**（最小扰动）。

---

## 3. 构建环境（M1/M2，已验证可用）

- WSL2 **Ubuntu-22.04**（WSL catalog 无 20.04）；源码 `~/rov-dev/ardupilot-external`
- 分支 `external-thrusters` @ tag `ArduSub-4.1.2`，基点 hash `2dd0bb7d`（**与实机逐字节一致**）
- ⚠️ 官方 `install-prereqs-ubuntu.sh` **在 22.04 会失败**（去装 Python2 的包）→ 已改**手动装最小依赖**：
  - apt：`build-essential ccache g++ gawk make wget pkg-config python3 python3-dev python3-pip python3-setuptools python3-wheel libtool libxml2-dev libxslt1-dev`
  - pip：`empy==3.3.4 pymavlink future lxml pexpect`（**empy 必须 3.3.x，4.x API 变了会崩**）
- 交叉工具链 **GCC 10.2**：`~/toolchains/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf`

**编译命令**（⚠️ `./waf` 的 shebang 要 `python`，而 22.04 只有 `python3` → **必须用 `python3 ./waf`**）：

```bash
cd ~/rov-dev/ardupilot-external
export ARM_TC=$HOME/toolchains/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf
python3 ./waf configure --board navigator --toolchain "$ARM_TC/bin/arm-none-linux-gnueabihf"
python3 ./waf sub -j4
# 产物 build/navigator/bin/ardusub（ARM EABI5 armhf ELF，约 1.9 MiB）
```

切 SITL：`python3 ./waf configure --board sitl && python3 ./waf sub`。两个 build 目录互不覆盖，回 navigator 需再 configure 一次。

---

## 4. 已完成的里程碑

| M | 状态 | 要点 |
|---|---|---|
| **M0** 基线/备份 | 完成 | 见 §2；工具 `fc_info.py`（版本+推进器参数+**uptime**）、`servo_monitor.py`（只读 8 路输出）、`fc_link.py`（锁定真飞控心跳） |
| **M1** 构建环境 | 完成 | 见 §3 |
| **M2** vanilla 编译+装机验证 | 完成 | 刷入**未改动**的自编译版 → heartbeat / 参数 / IMU 20Hz / 8 路恒 1500 全绿 → **glibc 与工具链兼容风险已排除** |
| **M3** C++ 实现 | 完成 | 编译通过，**从未刷机**。fork commit `d88e653`；diff 存 `patches/0001-external-8ch-thrust.patch`（7 文件 +214 行） |
| **M4** SITL 验收 | **进行中** | 见 §7 |

### M2 的关键订正（务必记住）

BlueOS 显示 `Successfully installed new firmware` **只代表写盘**，正在运行的进程仍然是旧的，直到点 **`RESTART AUTOPILOT`**。
而且我们编译的是同一个 tag，**版本号、git hash、`(STABLE)` 字样与官方完全一致，无法分辨新旧**。
→ **客观判据 = `fc_info.py` 打印的 autopilot uptime 归零**。M3 起固件会在开机发 STATUSTEXT 标记，以后可一眼确认跑的是哪份。

---

## 5. M3 实现要点（已对照真实 4.1.2 源码确认）

**真实输出链**：

```
output_armed_stabilizing()                     ← 派发器，按 FRAME_CONFIG 选混控
   └─ VECTORED_6DOF(2) → output_armed_stabilizing_vectored_6dof()   ← 本机 Heavy 走这条
        → 填 _thrust_rpyt_out[0..7]
output_to_motors()
   ├─ SHUT_DOWN / GROUND_IDLE → motor_out[i] = 1500     ← 强制中位
   └─ 否则 calc_thrust_to_pwm(_thrust_rpyt_out[i]) → rc_write()
```

**三处对原计划的修正**：

1. **切入点放在派发器顶部**拦截，三个混控函数**一行未动**（diff 小、便于日后 rebase 上游）
2. **只写 `_thrust_rpyt_out[]`，绝不在 MAVLink 回调里直接 `rc_write()`** → 下游 spool 门控仍在，**未解锁时输出被硬写成 1500 是结构性保证**，而不是靠自己写 if
3. 必须自己补 `_motor_reverse[i]` 与 `constrain_float(...,-1,1)`（混控最后一行做的事；否则 `MOT_n_DIRECTION` 会静默失效）；而 **`get_current_limit_max_throttle()` 在 4.1.2 里恒返回 `1.0f`（空壳）**，原计划担心的"总电流限制"无实质内容，无需处理

**接口**：`SET_ACTUATOR_CONTROL_TARGET` + 私有约定 **`group_mlx == 1`** = Motor1..8，`controls[i]` 取值 `[-1,1]`（归一化执行器命令，**不是牛顿**；分配器算出的 fᵢ 需经推力标定转到这层）

**校验**：`sysid == SYSID_MYGCS` / `target_system` 匹配 / `group_mlx == 1` / `MOT_EXT_ENABLE` 已开 / 8 路全部 finite 且在 `[-1,1]` → **整帧拒收，不做部分采纳**

**新参数**：`MOT_EXT_ENABLE`（0/1，**默认 0**，刷进去不改变现有行为）、`MOT_EXT_TMOUT`（ms，默认 500）

**failsafe**：

- 合法外部命令会刷新 `failsafe.last_pilot_input_ms` → **保留**原 pilot-input failsafe（否则进入外部模式后不再发 `MANUAL_CONTROL`，会被判 "Lost manual control" 而 disarm），而不是把它关掉
- 新增 `failsafe_ext_thrust_check()`（50Hz）：超时 → 归中位 + **disarm** + **锁存故障**；不恢复 mixer、不沿用上一条命令；**清 `MOT_EXT_ENABLE` 才解锁存**
- **故意不加 SITL 编译屏蔽**（原 `failsafe_pilot_input_check()` 带 `#if CONFIG_HAL_BOARD != HAL_BOARD_SITL`，在 SITL 里验不出来）

**改动文件**：`libraries/AP_Motors/AP_Motors6DOF.{h,cpp}`、`ArduSub/GCS_Mavlink.cpp`、`ArduSub/failsafe.cpp`、`ArduSub/Sub.h`、`ArduSub/ArduSub.cpp`、`ArduSub/system.cpp`（开机标记）

---

## 6. 约束条件（必须遵守）

**协作方式**

- **每一步先征求用户确认再推进**；分解要细
- **连机 / WSL / 编译类命令 → 给出精确可粘贴的命令，由用户在自己终端运行并把输出贴回**，助手判读后再给下一小步

**安全与时序**

- **编译不碰 ROV（随时可做）；刷机才碰 ROV** → 刷任何自编译固件必须排在 waypoint 收尾之后 + **用户显式同意** + 未解锁 + **物理隔离推进器**
- 隔离方式推荐 **拆螺旋桨**：电子舱与 ESC **共用同一块电池**，**不能靠拔电池隔离**（拔了 BlueOS 就没了，根本刷不了）
- **刷机后必须 `RESTART AUTOPILOT`**，并以 **uptime 归零**为判据
- **不得在 vanilla 装机验证通过之前刷改过的固件**（否则起不来时分不清是环境问题还是自己的 C++）
- 刷机前导出参数（已备份）、确认 `RESTORE DEFAULT FIRMWARE` 可用
- **不改动 `waypoint/`、`src/`、`config/`**；只动 `direct_thruster/` 与 WSL 里的 `ardupilot-external`
- 验收阶梯：**官方 SITL → 断桨干测 → 水下**。本项目的假 ArduSub（`waypoint/sim/`）**不能替代官方 SITL**

**环境**

- Windows 11 + PowerShell（主）；控制台 cp932 → Python 脚本内强制 stdout utf-8
- MAVLink **必须锁定真飞控心跳**（`autopilot != MAV_AUTOPILOT_INVALID` 且 `type != MAV_TYPE_GCS`），否则会锁到 BlueOS 服务（sys=0）导致参数一个都读不出来 → 用 `direct_thruster/fc_link.py` 的 `connect_fc()`
- WSL 源码可从 Windows 侧直接读写：`\\wsl$\Ubuntu-22.04\home\yaots\rov-dev\ardupilot-external\...`
- **别在 `/mnt/c` 下编译**（9p 文件系统极慢）
- git commit：`git -c user.name="Yaotseng3tsu" -c user.email="zeng@robot.t.u-tokyo.ac.jp"`，消息尾注 `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`

---

## 7. 当前进度：M4（SITL 验收）未完成

**第一次跑 0/11 通过，但诊断结论是：固件功能正常，失败的是测试脚本。**
证据：`[D2] SERVO3=[1500, 1740]` —— 外部命令**确实驱动了 Motor3**。

测试脚本 `direct_thruster/sitl_accept.py` 的四个问题（已部分修复）：

1. **SITL 默认是 6 推进器 Vectored 帧**，不是 Heavy → `SERVO7` 读数恒为 0，把"全中位"判据带崩 → **需要用 `FRAME_CONFIG=2` 起 SITL**
2. **两项测试之间空档超过 500ms，会让看门狗提前触发并锁存** → 后续测试被"正确地"拒绝，看起来像功能坏了（**这其实反证了看门狗在工作**）→ **每项测试前必须 `reset_ext()` 清锁存**
3. 开机 STATUSTEXT 抓不到（我们是 SITL 启动之后才连上的）→ 应降级为信息项，真正验证留到刷机后
4. 启动瞬态（读数 1000）污染了 A 项 → 需要 settle 延时

**已改但未完成**：`sitl_accept.py` 已加 `reset_ext()`、`arm_and_settle()`、自动探测真实电机通道（`MOTOR_CH`）；**测试主体序列尚未按新 helper 重写**。

**SITL 运行方法（已验证可用）**：

```bash
# SITL 与测试脚本必须在同一个 WSL 会话里（否则会话结束会杀掉 SITL）；python 要加 -u 不缓冲
mkdir -p /tmp/sitlrun && cd /tmp/sitlrun
setsid ~/rov-dev/ardupilot-external/build/sitl/bin/ardusub -S -I0 --model vectored_6dof \
    --defaults ~/rov-dev/ardupilot-external/Tools/autotest/default_params/sub.parm \
    > sitl.log 2>&1 < /dev/null &
timeout 200 python3 -u /mnt/c/bluerov2_mpc/direct_thruster/sitl_accept.py > accept.log 2>&1
```

### 下一步具体任务

1. **改完 `sitl_accept.py`**：用 `FRAME_CONFIG=2` 起 SITL（追加一个 defaults 文件）、每项测试前 `reset_ext()`、加 settle 延时、把开机标记降为信息项；然后重跑四项验收：
   - **A** 未解锁时发非零命令 → 8 路恒 1500
   - **B** 只给 Motor3 → 只有 SERVO3 离开中位，其余恒 1500
   - **C** 停发命令（心跳仍在）→ 归中 + disarm + 锁存 + 告警 STATUSTEXT
   - **D** 非法/越权拒收：错误 group / NaN / 超范围 / 锁存期间合法命令也拒绝 / 清 `MOT_EXT_ENABLE` 后恢复可控
2. SITL 全绿 → 提交并记录进 **`direct_thruster/PROGRESS.md`**（本线的进度写这里；仓库根的 `PROGRESS.md` 归 waypoint 线独占，别往那里追加）
3. 之后才谈：刷改版固件（**需用户同意 + 拆桨**）→ 断桨干测（用 `servo_monitor` 验 8 路独立）→ 写上位机 `external_thruster.py`（发 `SET_ACTUATOR_CONTROL_TARGET`，复用 `pseudo_stick` 的看门狗/退出归中+自动上锁范式）→ 水下

---

## 8. 线 A waypoint 现状（背景信息，勿改动其代码）

- 已真机下水。主力脚本：`go_forward.py`（高度 PID + 距离 PID + 航向 PID 三闭环 + **设定值节流**）、`z_move.py`、`altitude_hold.py`；另有 DVL 观测工具与 `go_waypoint.py`（点到点 4DOF）
- 关键实机结论：**`sign_z = -1`**（命令上浮 → 深度减小）；负浮力机器人需 `--u-bias` 恒定上推力才能悬停；池内定高用 **DVL altitude**（不是压力深度）；航向用 **ATTITUDE.yaw**（DVL 的 yaw 无罗盘会漂）；**设定值节流**是必要的（实测 surge 约 0.06 m/s 小于 v_cruise，不节流则 PID 饱和退化成开关控制）
- 已修复：深度源污染（VFR_HUD 恒 0 混入 `parse_depth`）、DVL 假丢底锁、**Cockpit 手柄抢 MANUAL_CONTROL**、航向跳变导致急旋、航向参考系平移（应平移目标而非打舵去追）、heading 加积分项 ki
- **遗留**：`waypoint/RESULTS.md` 的 §3 现场数据表仍然空着（下水实测数值未回填）

---

## 9. 关键坑清单（速查）

- 22.04 上老 `install-prereqs` 会去装 Python2 包并失败 → 手动装最小依赖
- `empy` 必须 3.3.x，不能用 4.x
- `./waf` 要用 `python3 ./waf` 来跑
- Navigator 的产物是 **ARM Linux ELF**（`build/navigator/bin/ardusub`），不是 `.apj`
- 别在 `/mnt/c` 下编译
- MAVLink 要锁定真飞控心跳，否则 sys=0 读不到参数
- 刷机后必须 RESTART 并看 uptime
- SITL 与测试脚本要在同一个 WSL 会话；python 加 `-u`
