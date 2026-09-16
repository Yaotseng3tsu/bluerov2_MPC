# Direct Thruster Control — 独立驱动 8 个推进器（修改并编译 ArduSub）

> **状态：v1**（2026-09-16，已吸收两段 ChatGPT 讨论）。独立研究子任务，**不改动 `waypoint/`、`src/`、`config/`**。
> 目标：跳过 MANUAL_CONTROL/伪手柄，让上位机对 8 个 T200 **逐个连续给归一化推力**，供 **MPC / RL / 自定义推力分配**。
> 路线（已定）：**保留 ArduSub 电机输出框架，替换"混控结果"这一层** —— 需修改并编译 ArduSub C++；MPC/分配器/上位机仍全用 Python。

> ## ⚠️ 与 waypoint 的时序边界（务必先读）
> **写文档 / 在 Ubuntu·WSL 上 clone 编译（哪怕编出 `ardusub`）= 不碰 ROV，随时可做、不影响 waypoint。**
> **一旦把任何自编译固件刷进 ROV（哪怕 M2 的 vanilla 未改版）= 替换了 waypoint 依赖的官方固件，会影响。**
> 因为 waypoint 走的是当前官方 ArduSub（MANUAL_CONTROL→混控）；刷机可能导致 ROV 起不来/丢 heartbeat、参数被重置需重应用、多一层不确定性。
>
> **时序规则：**
> 1. 先用**当前官方固件**把 waypoint 下水做完（vx 符号 / W2 辨识 / W5 航行）——**下水前不刷任何自编译固件**。
> 2. 本研究的**编译工作**（clone/装工具链/编 vanilla）不碰 ROV，可随时并行推进。
> 3. **真正刷机（M2 装机验证）排在 waypoint 下水之后**；刷前先导出参数、确认能一键 `Restore default ArduSub firmware`。

---

## 0. 项目组织：两份程序，不是把 Python 搬成 C++

```
开发电脑
├── bluerov2_MPC/                 现有 Python 项目 (不动)
│   ├── waypoint/ … MPC/分配器/DVL/安全范式
│   └── direct_thruster/          ← 本研究 (含未来的 external_thruster.py 上位机接口)
│
└── ardupilot-external/           另外 clone 的 ArduPilot 源码 (修改版 ArduSub)
    ├── ArduSub/  libraries/AP_Motors/
    └── build/navigator/bin/ardusub   ← 编译产物, 经 BlueOS 上传到 ROV
```

**运行关系**：

```
水面 (Python):  MPC → 推力分配器 → 8 路归一化命令
                          │  MAVLink (SET_ACTUATOR_CONTROL_TARGET, 现有网线/缆)
                          ▼
ROV (修改版 ArduSub, C++):  接收 → 校验控制权/有效期 → 替换混控结果 → 原有电机输出层
                          │
                          ▼
                    Navigator → ESC → 8×T200
```

C++ 只负责**接收、执行、保护**；Python 负责研究算法（改 MPC/分配/故障策略**不需要**重编 ArduSub，只有改机载接口/底层保护才重编）。

---

## 1. 关键决策

- **传输消息**：复用 MAVLink 标准 `SET_ACTUATOR_CONTROL_TARGET`（含 `float[8]`）作容器。
  ⚠️ 标准 group 0 = roll/pitch/yaw/throttle，**不等于 8 个独立电机**；需在修改版固件里**自定义约定**
  （如 `group_mlx=1` 表示 Motor1–8），收发两端一致。
- **接入层级**：替换 `AP_Motors6DOF::output_armed_stabilizing()` 里**产生 `_thrust_rpyt_out[]` 的混控部分**，
  **不是**在 MAVLink 回调里临时 `rc_write()`。这样才能复用后面的**电机反向、总电流限制、PWM 转换**。
- **归一化定义**：接口的 `[-1, 1]` 是**归一化执行器命令**，不是牛顿；分配器算出的 fᵢ 需经**推力标定**转到这层。
- **先 vanilla 后修改**：第一里程碑是"同版本、未改代码的 ArduSub 自编译后能在这台 ROV 正常启动"。

---

## 2. 里程碑（严格按序，不跳步）

| M | 目标 | 通过判据 |
|---|---|---|
| **M0** | 记录实机版本 + 导出参数 + 备份 | BlueOS 版本号、参数文件、输出功能/反向/PWM 范围都存档 |
| **M1** | 环境：WSL2 Ubuntu + clone 对应 tag + 交叉工具链 | `./waf configure --board navigator` 通过 |
| **M2** | **编译 vanilla（一行不改）→ 装机 → 验证 → 验证可回退** | 自编译 ardusub 装上后 heartbeat/IMU/depth/8路输出正常；且能一键恢复官方 |
| **M3** | 改 C++：8 路接收 + 替换混控 + 失效保护 | SITL 跑通 |
| **M4** | 编译修改版 → 装机 → 干测 | 断开推进器动力，`SERVO_OUTPUT_RAW` 验证 8 路独立、旁路了分配 |
| **M5** | 验收 + Python 接口 + 对接 MPC/RL | 8 路独立 + 失效停机四项验收通过（见 §7） |

> **M2 → M3 绝不能跳**：否则一旦"ArduSub 起不来"，无法区分是你的 C++ 错还是交叉编译环境错。

---

## 3. Step by Step

### Phase 0 · 记录与备份（M0，在 BlueOS 网页做）
- [ ] `Autopilot Firmware` → 记录**当前 ArduSub 版本号**（如 `ArduSub 4.x.x`）。
- [ ] `Autopilot Parameters` → **导出完整参数文件**（备份）。
- [ ] 记下：8 个推进器的输出功能（Motor1–8 对应哪路）、反向参数、PWM 范围、BlueOS 版本。
- [ ] 确认 BlueOS 有 **Restore default ArduSub firmware**（回退路径存在）。
> 也可用 MAVLink 只读查版本：连上后取 `AUTOPILOT_VERSION` / heartbeat 里的版本信息。

### Phase 1 · 构建环境（M1）
- **本机基线（M0 已确认，见 [M0_baseline.md](M0_baseline.md)）**：ArduSub **4.1.2**、Navigator、BlueOS **1.4.2**（Bullseye）、
  FRAME_CONFIG=2（Heavy）、SERVO1–8=Motor1–8/1100–1900。**checkout tag = `ArduSub-4.1.2`**；工具链 **GCC 10.2**（对 Bullseye）。
- [ ] Windows 用 WSL2 装 Ubuntu（**4.1.x 老分支建议 Ubuntu 20.04**，waf/依赖更匹配；22.04 也可试）：
  `wsl --install -d Ubuntu-20.04`（Python 控制程序继续留在 Windows）。
- [ ] clone 另一份源码（**含 submodules**）：
  ```bash
  mkdir -p ~/rov-dev && cd ~/rov-dev
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ardupilot-external
  cd ardupilot-external
  ```
- [ ] **切到实机对应的 tag** 并建分支（**别用 master**；下例仅示例版本）：
  ```bash
  git tag -l 'ArduSub-*' --sort=-v:refname | head -n 20
  git switch -c external-thrusters ArduSub-4.1.2      # ← 本机 M0 确认版本
  git submodule update --init --recursive
  ```
- [ ] 装官方构建依赖：
  ```bash
  Tools/environment_install/install-prereqs-ubuntu.sh -y
  . ~/.profile
  ```
- [ ] **交叉工具链**（x86 电脑生成 ARM Linux 程序；旧 BlueOS/Bullseye 用 **GCC 10.2**，避免运行库不兼容）：
  ```bash
  mkdir -p ~/toolchains && cd ~/toolchains
  wget -c https://developer.arm.com/-/media/Files/downloads/gnu-a/10.2-2020.11/binrel/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf.tar.xz
  tar -xf gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf.tar.xz
  ```

### Phase 2 · 编译 vanilla 并验证（M2，**门槛，勿跳**）
- [ ] configure（`--toolchain` 指向**含工具名前缀**的路径，不只是解压目录）：
  ```bash
  cd ~/rov-dev/ardupilot-external
  export ARM_TC="$HOME/toolchains/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf"
  ./waf configure --board navigator --toolchain "$ARM_TC/bin/arm-none-linux-gnueabihf"
  ./waf sub -j4
  ```
- [ ] 检查产物是 **ARM ELF**（不是 x86-64）：
  ```bash
  ls -lh build/navigator/bin/ardusub
  file build/navigator/bin/ardusub        # 期望: ELF 32-bit LSB ... ARM ...
  ```
- [ ] BlueOS：**上传固件前先确保未解锁、物理隔离推进器动力/断开 ESC 信号**。
  `Autopilot Firmware → Upload custom firmware → 选 ardusub → Install`。
- [ ] 验证 heartbeat 恢复、IMU/depth 遥测正常、参数可读、8 路输出正常（**"安装成功"字样不够**）。
- [ ] 验证能 `Restore default ArduSub firmware` 回官方。
> 备选：BlueOS 的 OpenVSCode 扩展可在 ROV 上直接 clone+编译（避开 PC→ARM 交叉编译兼容问题）；
> 但长期维护 fork 仍建议 Ubuntu 主机 + Git branch 作正式开发环境。

### Phase 3 · 修改 C++（M3）
要改的位置（**这是待开发内容，不是已存在功能**）：

| 文件 | 修改 |
|---|---|
| `ArduSub/GCS_MAVLink_Sub.cpp` | 接收外部 8 路命令；校验消息目标/来源/数值/协议约定（group_mlx=1） |
| `libraries/AP_Motors/AP_Motors6DOF.h` | 加 8 路命令缓存、有效标志、更新时间、setter 接口 |
| `libraries/AP_Motors/AP_Motors6DOF.cpp` | 外部控制启用时，用 8 路命令**替换混控结果** `_thrust_rpyt_out[]` |
| ArduSub 模式/控制源管理 | 何时接受外部控制；进入/退出/上锁时清理命令缓存 |
| `ArduSub/failsafe.cpp` 等 | 外部命令超时的安全动作，并与现有失联保护协调 |

**接入点（关键）** —— 当前正常输出链：
```
output_armed_stabilizing() → _thrust_rpyt_out[i]
        → output_to_motors(): motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i])
        → rc_write(i, motor_out[i]) → Navigator
```
目标结构：
```
output_armed_stabilizing()
   ├── NORMAL:   原混控 → _thrust_rpyt_out[]
   └── EXTERNAL: 你的 8 路命令 → _thrust_rpyt_out[]
        → 共同的输出保护/限制 → output_to_motors() → rc_write()
```
- ⚠️ **不能"填完数组立即 return"**：电机**反向处理**与**总电流限制**都在这个函数里，必须保留或重新接入。
- ⚠️ **pilot-input failsafe**：`failsafe_pilot_input_check()` 原本查驾驶输入更新时间。外部模式要**以通过校验的外部命令判断新鲜度**，
  不能因为"还在发 GCS 心跳"就当满足；**保留** GCS/漏水/电池等失效保护，别为了让新代码跑就关掉 failsafe。

### Phase 4 · 编译修改版 + 装机（M4）
- [ ] `./waf sub -j4`（增量编译，通常**不需要** `./waf clean`）；产物仍是 `build/navigator/bin/ardusub`。
- [ ] 装机前再次：未解锁 + 物理隔离推进器动力；BlueOS 上传安装。
- [ ] **干测**：断开推进器动力，读 `SERVO_OUTPUT_RAW` 验证"只改一路→只有对应电机变、其余中位"，确认旁路了分配矩阵。

### Phase 5 · 验收（M5，见 §7）

---

## 4. Python 上位机接口（第一版）

用 `SET_ACTUATOR_CONTROL_TARGET` 作传输容器；**前提是固件已实现本项目约定的 `group_mlx=1` 接收**（标准消息存在 ≠ 原版按此执行）。
下面只负责发送，不负责解锁/开启外部模式/安全：

```python
import math, time
from collections.abc import Sequence

def send_thrusters(conn, commands: Sequence[float]) -> None:
    """向修改版 ArduSub 发送 8 路归一化执行器命令 (需固件实现 group_mlx=1 接口)。"""
    values = [float(v) for v in commands]
    if len(values) != 8:
        raise ValueError("必须提供八个推进器命令")
    if not all(math.isfinite(v) for v in values):
        raise ValueError("命令不能含 NaN/Inf")
    if not all(-1.0 <= v <= 1.0 for v in values):
        raise ValueError("命令必须位于 [-1, 1]")
    conn.mav.set_actuator_control_target_send(
        time.monotonic_ns() // 1000,
        1,                      # 本项目约定组号 (非官方 Motor1-8 定义)
        conn.target_system, conn.target_component,
        values,
    )

# 例: 先发全中位
# send_thrusters(conn, [0.0] * 8)
```
- `[-1,1]` = 归一化执行器命令，**非牛顿**；分配器的 fᵢ 需经推力标定转到这层。
- 未来把连接/心跳/看门狗/退出归中+自动上锁范式从 `waypoint` 的 `pseudo_stick` 复用过来，作为直控层外壳。

---

## 5. 两个版本兼容坑（务必记住）

1. **源码 ↔ 构建工具**：旧 ArduSub 分支的 waf 可能不兼容较新 Python 环境 → 用匹配的 Ubuntu（如 22.04）。
2. **编译产物 ↔ 机载运行库**：较新编译器生成的程序在旧 Bullseye 上会因 glibc/运行库不匹配**起不来**
   → 旧 BlueOS 用 **GCC 10.2** 工具链。**"电脑上编译成功 ≠ ROV 上能启动"。**

---

## 6. 安全与回退

- **回退**：BlueOS `Restore default ArduSub firmware`；上传自定义前**必导出参数**，升/降级后按建议重新应用默认参数。
- **失控防护**：直写电机=没有混控/pilot failsafe 兜底 → **失效保护必须先于任何解锁实现并验证**。
- **外部命令过期的默认动作**（水槽测试）：**回中位 + 上锁 + 锁存故障，必须显式重新启用**；
  **不**恢复原 mixer、**不**无限保持上一条命令。
- 干测只短点动（气冷）；保留官方 MANUAL_CONTROL 链路作兜底。

---

## 7. 第一个验收目标（不是 MPC，而是"8 路独立 + 失联停机"）

| 测试 | 应确认 |
|---|---|
| 未解锁时发非零命令 | 输出保持安全中位 |
| 只改一路输入 | 只有对应电机变，其余中位 |
| 停发命令但心跳仍在 | 机载外部命令看门狗触发（归中+上锁+锁存） |
| 非法数值/错误来源/退出外部模式 | 不接受危险输出，不重放旧缓存 |

**测试阶梯**：官方 **SITL（跑真实 ArduSub 代码）** → 断开推进器的实机输出测量 → 受控水下。
⚠️ 本项目的"假 ArduSub"（`tests/sim_vehicle.py` / `waypoint/sim`）**不能替代 SITL** 这一步。

---

## 8. 备选路线（记录，暂不走）

- **Lua passthrough（不编译固件）**：把 8 个 `SERVOn_FUNCTION` 改为 `Script1–8`（Func 94–101），
  Lua `SRV_Channels:set_output_pwm` 直写。适合 onboard/低频；off-board 高频摄入动作困难。可秒回退，适合先做可行性/更新率验证。
- **自定义 `FRAME_CONFIG` 直通**（社区 MallARD `FRAME_CONFIG=8`）：评估是否有现成 1:1 frame。

---

## 参考来源（已核对）
- 官方 Build ArduSub <https://www.ardusub.com/developers/developers.html> · 输出映射 <https://ardupilot.org/sub/docs/common-rcoutput-mapping.html>
- Navigator waf 构建/工具链讨论 <https://discuss.bluerobotics.com/t/blueos-feedback-build-firmware-with-waf-for-the-navigator-flight-controller/13081> ·
  自定义固件装机 <https://discuss.bluerobotics.com/t/process-to-load-custom-ardupilot-build-into-blue-os-on-navigator-setup/20366>
- 独立控桨讨论 <https://discuss.bluerobotics.com/t/how-to-control-thrusters-independently/9870> · MAVSDK issue <https://github.com/mavlink/MAVSDK/issues/2145>
- Lua 直写输出 SRV_Channels PR <https://github.com/ArduPilot/ardupilot/pull/14366> · Scripting README <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/README.md>
