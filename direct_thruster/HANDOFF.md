# direct_thruster — 会话交接文档

> 更新：2026-09-16。用于把"修改并编译 ArduSub 实现独立控 8 桨"这条研究线交接给新会话。
> 新会话请先读本文件 + [README.md](README.md) + [M0_baseline.md](M0_baseline.md)。

## 1. 任务是什么
绕过 ArduSub 的 MANUAL_CONTROL/混控，让上位机对 **8 个 T200 逐个连续给归一化推力**（供 MPC/RL/自定义分配）。
路线（已定）：**保留 ArduSub 电机输出框架，替换"混控结果"这一层** → 需**修改并编译 ArduSub C++**；MPC/分配/上位机仍全 Python。
完整设计见 `README.md`（§1 方案对比、§3 改哪些 C++、§7 验收）。

## 2. 硬约束 / 工作方式（务必遵守）
- **每一步先征求用户确认再推进**；任何解锁/下水/**刷固件**前需显式同意。
- **不改动现有代码**：`waypoint/`、`src/`、`config/` 一律不碰；本研究只在 `direct_thruster/` + WSL 里的 `ardupilot-external`。
- **刷机时序**：先用当前官方固件把 waypoint 下水做完，**刷任何自编译固件排在其后**（见 README 顶部醒目提示）。编译不碰 ROV、随时可做；**刷机才碰 ROV**。
- **协作模式**：连机/WSL/编译命令 → 助手给出**精确可粘贴的命令**，**用户在自己终端跑、把输出贴回**，助手判读后给下一小步。分解要细。
- **环境**：Windows 11 + PowerShell（主）；编译在 **WSL2 Ubuntu-22.04** 里。仓库 `C:\bluerov2_mpc`，GitHub `Yaotseng3tsu/bluerov2_MPC`（main，持续 push）。
- **git commit**：`git -c user.name="Yaotseng3tsu" -c user.email="zeng@robot.t.u-tokyo.ac.jp" commit`，消息尾注 `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`。控制台 cp932 → 脚本内强制 stdout utf-8。

## 3. 实机基线（M0 已确认，见 M0_baseline.md）
- 固件 **ArduSub 4.1.2 (STABLE)**，git `2dd0bb7d`；Board **Navigator**（Blue Robotics），Submarine。
- BlueOS **1.4.2**（Bullseye，**32-bit armhf**）；有 `RESTORE DEFAULT FIRMWARE` 可回退。
- `FRAME_CONFIG=2`（Vectored 6DOF / BlueROV2 **Heavy**，8 桨）；**SERVO1–8 = Motor1–8**，未反向，PWM **1100–1900 / TRIM 1500**。
- 整机参数备份已入库：`direct_thruster/Submarine-4.1.2-STABLE-20260916-152651.params`。
- 决策：**版本 A —— 匹配当前 4.1.2 fork**。

## 4. 已完成到哪（进度）
- **M0 ✅**：版本/参数/映射记录 + 备份 + 决策 A。工具 `direct_thruster/fc_info.py`（只读查版本+推进器参数）。
- **M1 进行中**（全在 WSL，不碰 ROV）：
  - ✅ WSL2 已装；**Ubuntu-22.04** 已装（用户 `yaots`，主机 `tsengyao87`）。注：catalog 无 20.04，故用 22.04。
  - ✅ clone `~/rov-dev/ardupilot-external`，`git switch -c external-thrusters ArduSub-4.1.2`，
    HEAD=`2dd0bb7d4c`（**与实机 git hash 逐字节一致**），submodules 已 `--init --recursive`。
  - ✅ **构建依赖手动装**（官方 `install-prereqs-ubuntu.sh` 在 22.04 上报 Python2 包缺失 → **跳过**，只装编译必需）：
    `sudo apt install build-essential ccache g++ gawk make wget pkg-config python3 python3-dev python3-pip python3-setuptools python3-wheel libtool libxml2-dev libxslt1-dev`
    + `pip install --user empy==3.3.4 pymavlink future lxml pexpect`（**empy 必须 3.3.x，别用 4.x**）。
  - ⏳ **当前卡点 = M1-e2**：正在下载/验证 **GCC 10.2 交叉工具链**
    `~/toolchains/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf`（32-bit armhf，对 Bullseye）。
    下一动作：`.../bin/arm-none-linux-gnueabihf-gcc --version` 应显示 10.2。

## 5. 下一步（新会话从这里继续）
### M1-e2 收尾
- 确认工具链解压完成、`arm-none-linux-gnueabihf-gcc --version` = 10.2。若 ARM 下载链接失效，换镜像。

### M2 · 编译 vanilla（**不改一行代码，门槛，勿跳**）
在 `~/rov-dev/ardupilot-external`：
```bash
export ARM_TC="$HOME/toolchains/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf"
./waf configure --board navigator --toolchain "$ARM_TC/bin/arm-none-linux-gnueabihf"
./waf sub -j4
ls -lh build/navigator/bin/ardusub
file build/navigator/bin/ardusub          # 期望: ELF 32-bit LSB ... ARM ...
```
- configure/编译报错是**预期的趟坑**（老分支×新系统/py3.10）：缺模块补模块、报错逐个判读。常见：empy 版本、waf 的 python 兼容、submodule 缺失。
- 编译成功后 → **（在 waypoint 下水之后）** 经 BlueOS `Autopilot Firmware → Upload custom firmware` 装机，验证 heartbeat/IMU/depth/8 路输出正常 + 能 `Restore default`。**"电脑编译成功 ≠ ROV 能启动"**（GCC10.2 就是为此选的）。

### M3+ 才改 C++
见 README §3：改 `AP_Motors6DOF.{h,cpp}`（在 `output_armed_stabilizing()` 用外部 8 路命令替换 `_thrust_rpyt_out[]`，**保留电机反向/总电流限制**）、`GCS_MAVLink_Sub.cpp`（收 `SET_ACTUATOR_CONTROL_TARGET`，自定义 `group_mlx=1`=Motor1–8）、`failsafe.cpp`（外部命令超时→归中+上锁+锁存）。
验收先做"8 路独立 + 失联停机"（非 MPC），官方 SITL → 断桨干测 → 水下；**本项目假 ArduSub 不能替代官方 SITL**。

## 6. 关键坑清单
- 22.04 上老 `install-prereqs` 装 Python2 包会失败 → 已改手动装最小依赖。
- `empy` 必须 3.3.x（4.x API 变、会崩）。
- Navigator 产物是 **ARM Linux ELF**（`build/navigator/bin/ardusub`），不是 `.apj`。
- **别在 `/mnt/c` 下编译**（9p 极慢）→ 用 Linux 家目录 `~/rov-dev`。
- 刷机前导出参数（已备份）、确认可 Restore；未解锁 + 物理隔离推进器动力再刷。
