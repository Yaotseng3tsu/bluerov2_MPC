# Direct Thruster Control — 研究文档（绕过 ArduSub 混控，独立驱动 8 个推进器）

> **状态：v0 草稿**（2026-09-16）。本文件夹是**独立研究子任务**，不改动 `waypoint/` 及现有任何代码。
> 目标是评估并实现"跳过 MANUAL_CONTROL/伪手柄，直接对 8 个 T200 逐个连续给 PWM"，供 **RL / 自定义分配矩阵 / 直接控制**。
> ⚠️ 待办：把两段 ChatGPT 讨论（"比较 BlueROV2 控制方式"等）的结论贴进 §7，与本文对齐后升级到 v1。

---

## 0. 背景与动机

- **现状（现有研究）**：`waypoint/` 与整个 `bluerov2_mpc` 走 **ArduSub 标准链路** ——
  `MANUAL_CONTROL(x/y/z/r)` → ArduSub **固定 Heavy 分配矩阵**（motor mixer）→ 8 个 ESC。
  已验证：轴映射标准 Heavy、推力权限偏低（u=1.0→±80µs）。
- **痛点**：官方**不提供 per-thruster 的 Python 接口**；RC_OVERRIDE 与 MANUAL_CONTROL 都要经过混控分配，
  无法独立、连续、协调地驱动每个桨 —— 这正是 RL（动作空间=8 维推力）或自定义控制分配所需要的。
- **本研究**：另开一条控制路径，独立文件夹推进，**不干扰现有 MANUAL_CONTROL 链路**（保留其为安全兜底）。

---

## 1. 方案对比（先评估，再决定是否 fork 固件）

| 方案 | 独立/连续控制 | 需改固件? | 上位机接口 | 延迟/频率 | 适合 RL? | 备注 |
|---|---|---|---|---|---|---|
| **A. `MAV_CMD_DO_MOTOR_TEST`** | 单桨、逐个 | 否 | pymavlink | 有超时、低频 | ❌ | 只适合测试单桨方向/健康 |
| **B. `MAV_CMD_DO_SET_SERVO`** | 单通道 | 否 | pymavlink | 每周期可能被混控覆盖 | ❌ | 需把 `SERVOn_FUNCTION` 设 Disabled 才不被覆盖；不协调 |
| **C. Lua 脚本 + Script 输出** | 8 桢独立 | **否**（仅改参数+脚本） | 板载 Lua 读取 | 受 Lua 调度限制 | 🟡 | ArduSub 支持 Script1–16（Func 94–109）；`SRV_Channels:set_output_pwm`。**onboard/低频控制的最轻方案** |
| **D. 自定义 `FRAME_CONFIG`** | 输入→电机直通 | 否 | MANUAL_CONTROL | 标准 | 🟡 | 社区 MallARD 用 `FRAME_CONFIG=8` 做过 1:1 直通；需确认 ArduSub 现成 frame 是否满足 |
| **E. Fork + 编译 ArduSub** | **8 桢独立、连续、协调** | **是** | 自定义/`SET_ACTUATOR_CONTROL_TARGET` | 低延迟、控制率 | ✅ | 最强但最重；**off-board RL 的正解** |

**关键判断**：
- 若 RL/控制器**跑在板载**或对更新率要求不高 → **先试 C（Lua）**，零编译、可回退，最快出可行性结论。
- 若 RL/控制器**跑在上位机、需要控制率下发 8 维动作** → Lua 难以高频摄入 off-board 动作 → **走 E（fork）**，
  用一条 MAVLink 消息把 8 个归一化推力直送固件、直写电机输出。**这也是本项目倾向的路线。**

> 建议路线：**先花半天验证 C（Lua）的实际更新率/延迟**（作为 baseline 和回退），
> 再投入 E（fork）。两者不冲突，C 的参数改动也可秒回退。

---

## 2. 路线 E（fork + 编译）—— Step by Step

> 全程**先 SITL 后真机**；每一步先与用户确认再推进；**保留官方固件备份**可随时回退。

### E1 · 工具链与源码
- [ ] 克隆 ArduPilot 源码（**ArduSub-stable** 分支）**含 submodules**：
  `git clone -b ArduSub-stable --recurse-submodules https://github.com/ArduPilot/ardupilot`
- [ ] 装 waf 构建依赖：`Tools/environment_install/install-prereqs-ubuntu.sh -y`（在 WSL/Linux 上）。
- [ ] 目标板：**Navigator**（RPi4 上的 Linux target），waf board = `navigator`。

### E2 · 先在 SITL 打通
- [ ] `./waf configure --board sitl && ./waf sub`；`Tools/autotest/sim_vehicle.py -v ArduSub` 起 SITL。
- [ ] 在 SITL 验证"直写 8 输出"的改动逻辑，**完全不碰真机**。

### E3 · 定位混控/输出代码（研究点，需读源码确认）
- [ ] Sub 的电机输出：`ArduSub/` + `libraries/AP_Motors/AP_Motors6DOF.*`（6DOF 分配矩阵）。
- [ ] 找到"分配矩阵 → 各电机推力 → `SRV_Channels` 写 PWM"的落点（`output_to_motors` / `output_armed` 一类）。
- [ ] 确认电机输出通道与 `SERVOn_FUNCTION`（Motor1..8）的映射。

### E4 · 实现"直写通道"
- [ ] 选消息：优先复用 **`SET_ACTUATOR_CONTROL_TARGET`**（MAVLink 标准，含 8 个归一化输出 group），
  或自定义消息；上位机按此发 8 维动作。
- [ ] 加一个**模式门**（param 或消息里的标志）：激活时**用消息里的 8 值直接写电机输出、跳过分配矩阵**；
  未激活时行为与官方一致（保留 MANUAL_CONTROL）。
- [ ] **失效保护（必做）**：超过 X ms 未收到新动作 → 8 输出归中位 + 报警（对齐 `pseudo_stick` 的看门狗范式）。
- [ ] 保留 arm/disarm、ESC 死区、限幅逻辑。

### E5 · 编译
- [ ] `./waf configure --board navigator && ./waf sub` → 产出 Navigator 固件（`build/navigator/bin/ardusub`）。

### E6 · 刷入 Navigator（BlueOS）
- [ ] 通过 **BlueOS 自定义固件上传**（Pirate/开发者模式的 firmware upload）刷入自编译固件。
- [ ] **先备份当前官方固件**；确认可一键回退官方版。

### E7 · 上位机接口（与现有安全范式对齐）
- [ ] Python 端按控制率发送 8 维动作消息；复用 `pseudo_stick` 的连接/心跳/看门狗/退出归中+自动上锁范式。
- [ ] 提供"直控层"替换 `pseudo_stick.send` 的等价物（新模块，不改旧文件）。

### E8 · 安全与验证阶梯
- [ ] SITL 逐桨 → 干测（出水、单桨短点动、读 `SERVO_OUTPUT_RAW` 验证直写生效且旁路了分配）→ 水下。
- [ ] 失效保护实测（断消息 → 归中）；ESC 死区/冷却/急停预案。

---

## 3. 路线 C（Lua）—— 快速可行性验证（推荐先做）

- [ ] 把 8 个推进器的 `SERVOn_FUNCTION` 改为 `Script1..Script8`（Function ID **94–101**）。
- [ ] 写 Lua 脚本用 `SRV_Channels:set_output_pwm(<func>, <pwm>)`（或 `set_output_pwm_chan_timeout` 带超时）
  周期性写 8 输出。
- [ ] 测**实际更新率 / 抖动 / 端到端延迟**；结论决定是否够 RL 用。
- [ ] 动作来源：板载脚本内生成，或从 param/命名值/串口摄入 off-board 动作（评估瓶颈）。
- 优点：**零编译、秒回退**（改回 `SERVOn_FUNCTION` 即恢复官方）。

---

## 4. 与现有项目的关系

- **独立**：新文件夹 `direct_thruster/`，不改 `waypoint/`、`src/`、`config/`。
- **可复用**：DVL 读取（`waypoint/dvl_stream.py`）、状态估计、安全范式（看门狗/退出上锁）都可复用为直控层的外壳。
- **兜底**：官方 MANUAL_CONTROL 链路与 `waypoint/` 保持可用，作为直控失败时的安全回退。

---

## 5. 风险与回退

- **刷错/变砖**：务必先备份官方固件，确认 BlueOS 一键回退路径。
- **失控**：直写电机=没有混控/失效兜底，**失效保护必须先于任何解锁实现并验证**。
- **热管理**：干测只短点动（气冷）。
- **可维护性**：fork 后需跟踪上游 ArduSub 更新（记录改动 diff、基于 tag 分支）。

---

## 6. 里程碑（建议顺序）

1. **M0** 本文档 + 决策：先 Lua 验证还是直接 fork（见 §7 待与 ChatGPT 对齐）。
2. **M1** 路线 C（Lua）可行性：8 桢直写 + 更新率/延迟报告。
3. **M2** 路线 E SITL：自定义消息直写 + 失效保护，SITL 跑通。
4. **M3** 路线 E 真机：Navigator 编译刷入 + 干测逐桨验证旁路分配。
5. **M4** 直控层 Python 接口 + 与 RL/控制器对接。

---

## 7. 待办：与 ChatGPT 讨论对齐（reconcile）

> 我（助手）无法直接读取以下分享链接（页面 JS 渲染，抓取到的是登录壳）。请把两段讨论的**关键结论**贴过来，
> 我据此把本文从 v0 升到 v1（尤其是：最终选定的方案、具体 MAVLink 消息、涉及的源码文件、编译/刷机注意点）。

- 链接1（"比较 BlueROV2 控制方式"）：<https://chatgpt.com/s/t_6aa9e72404d88191987e892ba2c841da>
- 链接2：<https://chatgpt.com/s/t_6aa9ec91260c8191a6bf2e5c64bf3845>
- [ ] 贴入结论 → 确认 Lua-first vs fork-first
- [ ] 确认选用的消息（`SET_ACTUATOR_CONTROL_TARGET` / 自定义）
- [ ] 确认目标板/BlueOS 刷机的具体步骤是否与本文一致

---

## 参考来源（已核对）

- ArduSub 独立控桨讨论：<https://discuss.bluerobotics.com/t/how-to-control-thrusters-independently/9870> ·
  <https://discuss.bluerobotics.com/t/independent-control-of-a-motor/21558> ·
  MAVSDK issue <https://github.com/mavlink/MAVSDK/issues/2145>
- Lua 直写输出：ArduSub 输出映射 <https://ardupilot.org/sub/docs/common-rcoutput-mapping.html> ·
  SRV_Channels 超时覆盖 PR <https://github.com/ArduPilot/ardupilot/pull/14366> ·
  Scripting README <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/README.md>
- 自定义 frame 直通（MallARD, FRAME_CONFIG=8）：<https://github.com/EEEManchester/MallARD_Pixhwak>
- 编译/刷机：官方 Build ArduSub <https://www.ardusub.com/developers/developers.html> ·
  Navigator 自定义固件 <https://discuss.bluerobotics.com/t/process-to-load-custom-ardupilot-build-into-blue-os-on-navigator-setup/20366> ·
  waf 构建讨论 <https://discuss.bluerobotics.com/t/blueos-feedback-build-firmware-with-waf-for-the-navigator-flight-controller/13081>
