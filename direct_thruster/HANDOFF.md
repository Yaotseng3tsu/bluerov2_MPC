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
| **M4** SITL 验收 | 完成 | 2026-10-07 官方 SITL **14/14 通过**（A 未解锁不动桨 / B 八路逐一独立 + 推力标度 + 负向 / C 看门狗归中+disarm+告警+锁存 / D 四类拒收 + 按流程恢复）。见 §7 |

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

- **编译不碰 ROV（随时可做）；刷机才碰 ROV** → 刷任何自编译固件必须排在 waypoint 收尾之后 + **用户显式同意** + 未解锁
- **拆桨隔离这一条已于 2026-10-06 经用户决定放宽**（判断是风险没那么高）。仍然成立的事实：电子舱与 ESC **共用同一块电池**，所以**不能靠拔电池隔离**（拔了 BlueOS 就没了，根本刷不了）。助手建议首轮干测用小幅度短点动（±0.1、每次 1 秒）而不是 0.6：直控层没有混控兜底，单桨满推时整机会在台面上移动
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
- **两条线各占一个 git worktree（2026-10-07 起）**：`C:luerov2_mpc` 归 waypoint，
  **`C:luerov2_mpc_dt` 归 direct_thruster**（本线的一切都在这里做）。共用同一个 `.git`，
  所以分支 / 远端 / 历史是同一套；一个分支同一时间只能被一个 worktree 检出。
  > 原本定的是"单工作目录、main 上串行、不开 worktree"，理由是瓶颈不是 git 而是用户本人。
  > 2026-10-07 推翻：用户为两条线各开了一个会话，当天踩了两次 —— ① 另一个会话在共用目录里
  > 切了分支，本线的提交落到了 waypoint 的分支上；② 目录切回 waypoint 分支后，
  > `direct_thruster/` 下的新文件在工作树里**不存在**，跑脚本直接 `No such file`。
  > 两条线的文件物理上无法同时在场，这不是意外而是单工作目录的必然结果。
- **每次 commit / push 前先 `git branch --show-current` 核对**。不对就停下来问，
  **不要自作主张切分支** —— 切分支会改掉另一个会话正在读的文件内容。

---

## 7. M4（官方 SITL 验收）— ✅ 14/14 通过（2026-10-07）

一条命令复现：

```bash
# SITL 与测试脚本必须在同一个 shell 会话里，会话一结束 SITL 就会被杀 ——
# sitl_run.sh 已把这件事连同 FRAME_CONFIG=2、SERVOn_FUNCTION、擦 eeprom 一起封好。
bash /mnt/c/bluerov2_mpc/direct_thruster/sitl_run.sh
```

| 项 | 结果 |
|---|---|
| **A** 未解锁时发 0.6×8 | 8 路恒 1500 ✅（结构性保证：`output_to_motors()` 在 SHUT_DOWN 下硬写 1500） |
| **B1–B3** 单路 / 标度 / 负向 | SERVO3 → 1740（=1500+0.6×400）、−0.6 → 1260 ✅ |
| **B4** 八路逐一扫描 | 八路各自给 0.5 → 对应 SERVO 到 1700、其余七路恒 1500 ✅ **（这条就是本线的目标本身）** |
| **C1–C4** 看门狗 | 归中 / 自动 disarm / `Lost external thrust commands` / 锁存期间合法命令也被拒 ✅ |
| **D1–D5** 拒收与恢复 | 错 group / NaN / 越范围 / 错 target_system 四类整帧拒收；清 `MOT_EXT_ENABLE` 再开后恢复可控 ✅ |

**SITL 通过证明了什么**：接口约定（`group_mlx=1`）、整帧校验、绕过混控的替换点、`_motor_reverse` 与限幅、
spool 门控、看门狗+锁存+恢复流程，这些**逻辑**在真实 ArduSub 代码里按设计工作。

**SITL 没有证明的**（刷机后仍须验）：Navigator 板上的真实 PWM 输出时序、ESC 响应、推力标定
（`[-1,1]` → 牛顿）、以及任何水动力。

### 走到 14/14 用了四轮，根因全部在测试台与环境，固件 C++ 一行未改

| 轮次 | 结果 | 根因 |
|---|---|---|
| 1 | 0/11 | SITL 默认 6 推进器帧 → Motor7/8 未使能、SERVO7/8 恒 0，把所有"全中位"判据整体带崩 |
| 2 | 11/14 | `reset_ext()` 竞态（见下）；拒收类测试落回原混控而 SITL 无人喂 RC；窗口切换时上一窗口残留报文串扰 |
| 3 | 11/14 | **墙钟跳变**（见下）伪造出"链路空洞" |
| 4 | 13/14 | **`SERVO8_FUNCTION` 不是 Motor8**（见下）→ Motor8 没有输出通道 |

三个值得单独记住的坑：

1. **`reset_ext()` 的竞态**。固件里"解锁存"和"清残留命令"是两条独立路径：`clear_external_fault()`
   只在 `MOT_EXT_ENABLE==0` 时被 50Hz 检查调用，而清 `_ext_have_cmd` 的 `clear_external_thrust()`
   **只在看门狗真正触发时**调用。所以不能直接 `ENABLE 0→1`：那一瞬 `external_thrust_timed_out()`
   立即为真 → 20ms 内当场重新锁存，后续测试全被"正确地"拒绝。
   正确顺序：**先让看门狗在 ENABLE 还开着时打一次**（它会清掉 `_ext_have_cmd`），再 `0→1`。
2. **WSL2 的墙钟会前后跳 ±7.9 秒**（实测，约每 15 秒一次）。用 `time.time()` 量时间间隔会量出
   不存在的"数秒空洞"，进而伪装成功能失败。**计时一律用 `time.monotonic()`。**
   判别方法：同时统计"收包间隔"和"自己循环的间隔"——两者都大 = 真卡住；只有前者大 = 链路问题；
   墙钟与单调钟的差 = 时钟跳变。本脚本三项都打在每条判据的详情里。
3. **SITL 必须钉死 `SERVO1..8_FUNCTION = 33..40`**（`k_motor1..k_motor8`）。`add_motor_num()` 只用
   `set_aux_channel_default()` 装**默认值**，被 `sub.parm` 显式设过的 `SERVOn_FUNCTION` 会赢 →
   那个电机**根本没有输出通道**，而对应 SERVO 显示的是别的功能、恰好停在 1500，看起来就像
   "这一路驱动不了"。实机 M0 基线本来就是 SERVO1–8 = Motor1–8，所以这也是让 SITL 对齐实机。

验收台由三个文件组成：`sitl_ext.parm`（SITL 追加默认参数）、`sitl_run.sh`（一键起停）、
`sitl_accept.py`（14 条计分项）。**前置条件一律硬 abort**——改版参数在位 / `FRAME_CONFIG==2` /
`SERVO1-8_FUNCTION==33..40` / 电机通道=={1..8} / 基线全中位。第一轮 0/11 的教训就是前提错了还
照常跑完，产出一堆看不懂的 FAIL。

### 下一步

1. **M5 刷改版固件**（**需用户显式同意**；排在 waypoint 收尾之后；拆桨隔离这一条用户已于
   2026-10-06 放宽）。刷完必 `RESTART AUTOPILOT`，判据 = `fc_info.py` 的 uptime 归零；
   这次还能用开机 STATUSTEXT `EXT-THRUSTER build...` 直接确认跑的是哪份。
2. **干测**：`servo_monitor.py` 看 8 路输出，重跑 B4 那套逐一扫描。
   建议首轮用小幅度短点动（±0.1、每次 1 秒）而不是 0.5——直控层没有混控兜底，单桨满推时整机会在台面上移动。
3. ~~上位机 `external_thruster.py`~~ **已完成并在 SITL 验过**（2026-10-07，四步全过：未解锁不动桨 /
   单路 1620 / 八路扫描 / `u_max` 限幅）。干测时直接用：
   `--endpoint udpin:0.0.0.0:14550 --arm --u-max 0.1 --sweep --thrust 0.1 --dwell 1.0`
4. **推力标定**：把分配器的 fᵢ（牛顿）映射到接口的 `[-1,1]`。
5. 之后才是水下。

### 闭环层的设计要求（用户 2026-10-07 提出，尚未实现）

**从池底起步必须先用压力深度，升到一定高度后才切 DVL。** DVL 贴底时低于最小量程、
拿不到稳定底锁，`altitude` 不可用；现有 `go_forward.py` 在那种情况下会直接判
"DVL 丢底锁/超龄"退出 —— 也就是说**它压根起不了飞**。

怎么接到现有估计器上（`waypoint/altitude_estimator.py` + `nav_state.py`，两条线共享）：

| 通道 | 现状 | 要做的 |
|---|---|---|
| **上升率** | 已经是压力源（`GLOBAL_POSITION_INT.vz` = EKF 垂速，由压力计+IMU 得出），贴底照样有 | 不用动 |
| **位置/绝对值** | 只有 `update_alt(DVL altitude)` 一条路，贴底拿不到 → `alt_ok=false` | 起飞段改用**相对起飞点的 Δdepth**（压力计），够用来做"上升 N 米" |
| **源切换** | — | DVL 的 `altitude` 有效且 > 阈值、且连续 N 帧稳定 → 切过去。切换时算出池底深度 `d_bottom = depth + alt`，之后两个源可以互校 |

**切换要走估计器已有的 `reset_delta` 机制**（跳变再基准：发布 Δ 让控制器平移目标，
而不是去消那个阶跃）。这个钩子本来就是为这类情况设计的，不要另造一套。

两个坑：

- **压力深度的绝对值不可靠**。`nav_state.py` 的注释已经写明：水面基准会被
  `update_calibration()` 重新归零导致阶跃，所以那边**刻意只用速率不用绝对深度**。
  起飞段用 Δdepth 是**相对量**、窗口又短，风险小一些，但归零一旦发生仍会污染 ——
  需要把它当跳变检出来。
- **切换阈值要实测，不要假设**。A50 标称最小高度 ~0.1m，但实际可用高度更高。
  阈值 + 迟滞 + 连续有效帧数这三个量都得在池里标。


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
- **WSL2 墙钟会前后跳数秒** → 任何计时一律用 `time.monotonic()`，别用 `time.time()`。
  根因（2026-10-07 已修）：Windows 的 `W32Time` 服务停着、宿主时钟慢了 7.75 秒，而 WSL 里
  `systemd-timesyncd`（校到 NTP）和 `/dev/ptp_hyperv`（拉宿主时间）两个授时源来回拉锯。
  修法是校准**宿主**：管理员 PowerShell 跑 `Start-Service W32Time` +
  `w32tm /config /manualpeerlist:"time.windows.com,0x9" /syncfromflags:manual /update` +
  `w32tm /resync /force`，再 `wsl --shutdown`。
  查法：`w32tm /stripchart /computer:time.windows.com /samples:5 /dataonly` 看宿主偏移；
  WSL 里比较 `time.time()` 与 `time.monotonic()` 的相对漂移看跳变。
  **但代码该用单调时钟还是要用** —— 环境随时可能再坏，不该靠环境正确才成立。
- SITL 要钉死 `SERVO1..8_FUNCTION=33..40`，否则 `sub.parm` 的显式值会抢走电机的输出通道
