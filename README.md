# BlueROV2 (R4) MPC 控制 — 从伪手柄指令到单维度 MPC 闭环

用纯 Python + `pymavlink`,不依赖 ROS、不跑仿真环境、不做 GUI,
一步步让 BlueROV2 R4 **动起来**,并最终用 **MPC 替代 PID** 完成**深度(Heave)单维度定深/定位**。
参考 [HKPolyU-UAV/bluerov2](https://github.com/HKPolyU-UAV/bluerov2) 的思路,但做成**极简、可分步验证**的版本。

> **本项目的最低目标**:通过代码给出归一化运动指令(类似 `motors.set_forward` 的伪手柄指令),让机器人动起来。
> **本项目的最终目标(第一阶段)**:深度单维度闭环,先用 PID 做基线,再换成 MPC,对比性能;后续预留 RL 接口。

---

## 0. 为什么先做"深度(Heave)"这一个维度

来自参考资料 `Stationkeep Discussion.docx` 的关键结论:

- 视频里的 Station Keeping 本质是 **DVL 作为观测源 + ArduSub 内置 Position Hold(PID)**,不是 DVL 直接控推进器。
- **水平面(surge/sway/yaw)是难点**:没有 DVL 就没有可靠的水平速度/位置反馈;而且你的 **yaw/compass 不太可靠**,会把速度旋转到错误方向。
- **深度是最容易、最可靠的单维度**:BlueROV2 的压力传感器直接给出深度,反馈干净、不依赖 DVL 和 yaw,风险最低。

因此第一阶段选择 **Heave(垂直/深度)** 作为第一个闭环维度。等这一维跑通,再考虑扩展到 yaw / 水平面 / 多自由度 / RL。

---

## 1. 总体路线图(Phase 0 → Phase 6)

| 阶段 | 名称 | 目标 | 交付物 | 是否需要下水 |
|---|---|---|---|---|
| **P0** | 环境与连接 | venv + pymavlink,能收到 heartbeat,能读深度 | `src/link.py`、连接自检脚本 | 否(桌面即可) |
| **P1** | 开环驱动(**最低目标**) | 代码发归一化 `MANUAL_CONTROL`,电机按指令动;确定 z 中位与正负号 | `src/pseudo_stick.py` | 建议先干测/系留 |
| **P2** | 状态反馈 | 稳定 10–50 Hz 读取深度并时间对齐,形成 observation | `src/state.py` | 否 |
| **P3** | 系统辨识 | 采集阶跃响应,拟合深度方向一阶/二阶模型 | `src/sysid.py`、`config/depth_model.yaml` | 是(水池) |
| **P4** | PID 基线 | 外环 PID 定深,作为对比基线 | `src/pid_depth.py` | 是(水池) |
| **P5** | **MPC 闭环** | 用 MPC 替代 PID 定深,和基线对比 | `src/mpc_depth.py` | 是(水池) |
| **P6** | 扩展(可选) | yaw / 水平面 / 多 DOF / RL | 待定 | 是 |

> **监管协议**:每个阶段结束都必须经过你的确认才进入下一阶段。每阶段的目的、命令、结果、问题都会记录在 [`PROGRESS.md`](PROGRESS.md),并做一次 git 提交。详见第 8 节。

### 1.1 仿真环境(SITL)—— 无真机时的开发闭环

参考 [HKPolyU-UAV/bluerov2](https://github.com/HKPolyU-UAV/bluerov2) 的"先仿真后真机"思路,本项目自带一个**极简软件在环(SITL)**,让 P1–P5 全流程在没有机器人时就能开发验证:

- [`src/plant.py`](src/plant.py):BlueROV2 **深度(heave)动力学模型**(有效质量+线性/二次阻尼+剩余浮力+推力,RK4 积分)。它**同时**给 SITL 当"真值"、给 MPC(P5)当"内部预测模型"。参数在 [`config/depth_model.yaml`](config/depth_model.yaml)。
- [`tests/sim_vehicle.py`](tests/sim_vehicle.py):**闭环 MAVLink SITL**,像真 ArduSub 一样接收 `MANUAL_CONTROL` → 积分动力学 → 回传深度遥测。

**关键点:控制代码对仿真和真机使用完全相同的 MAVLink 接口**,只是连接对端不同。

```text
[控制器: link/pseudo_stick/pid/mpc]  udpin:0.0.0.0:14550
        │  MANUAL_CONTROL ↓            ↑ HEARTBEAT + 深度
[SITL: tests/sim_vehicle.py]  bind :14551 → 遥测发往 127.0.0.1:14550
        (真机时: 换成 BlueOS 的 UDP endpoint,控制器代码不变)
```

运行(两个终端):
```powershell
cd C:\bluerov2_mpc
.\.venv\Scripts\python.exe tests\sim_vehicle.py             # 终端A:闭环 SITL
.\.venv\Scripts\python.exe -m src.pseudo_stick --demo-heave  # 终端B:下潜2s→上浮2s→停
# P0 连接自检(SITL 用 --demo 正弦模式):
.\.venv\Scripts\python.exe tests\sim_vehicle.py --demo
.\.venv\Scripts\python.exe -m src.link --check --seconds 12
```

> ⚠ 仿真里 `MANUAL_CONTROL` 的 z 采用 `0..1000、500 中位、+ 下潜` 约定;**真机的 z 中位与正负号仍需 P1 实测确认**(见第 3 节)。

---

## 2. 环境配置

### 2.1 硬件 / 网络前提(BlueOS 侧)

- BlueROV2 **R4**,已升级 Navigator 飞控 + BlueOS。
- 上位机(本 Windows 电脑)与 ROV 同网段,ROV 默认 `192.168.2.2`,上位机常见为 `192.168.2.1`。
- 在 **BlueOS → Pirate Mode → MAVLink Endpoints** 新建**专用于本项目的 UDP endpoint**:
  - **控制/遥测(本项目)**:`UDP Client` → 上位机 IP,端口 **`14550`**(与 QGC/Cockpit、以及现有 logger 的 `14560` 区分开,避免抢包)。
  - 如果探测不到消息,尝试把路由器切到 **MAVLinkServer** 模式(会把所有消息转发给所有客户端)。
- **Windows 防火墙**放行 Python 接收 UDP `14550`。
- 切换 MAVLink 路由属于高级操作:**务必先在未解锁(disarmed)状态**下确认 Cockpit 状态/视频/遥测/failsafe 正常,再做水池测试。

### 2.2 上位机软件

```powershell
cd C:\bluerov2_mpc
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

依赖分层(见 [`requirements.txt`](requirements.txt)):

- **P0–P4 必需**:`pymavlink`、`numpy`、`pyyaml`
- **P3 分析/绘图**:`matplotlib`、`scipy`
- **P5 MPC**:`casadi`(参考论文用 CasADi/IPOPT);可选 `do-mpc` 做更高层封装
- **P6 RL(可选)**:`gymnasium`、`stable-baselines3`(后期再装)

---

## 3. 数据链路与命令接口

```text
本项目 (src/) ──MANUAL_CONTROL (10–25 Hz, 归一化 ±1000)──┐
                                                        BlueOS MAVLink router ──> ArduSub(Navigator)
ArduSub ── 深度遥测 (SCALED_PRESSURE2 / VFR_HUD) ────────┘
                                                        └──> UDP 14550 ──> src/ (pymavlink udpin)
```

- **发指令(伪手柄)**:`MANUAL_CONTROL`,字段含义(ArduSub):
  - `x` = 前进 surge,`y` = 右移 sway,`r` = 偏航 yaw:范围 **-1000..1000**,`0` = 中位。
  - `z` = 垂直 heave / 油门:**中位与正负号需在 P1 用极小指令实测确认**(常见约定 `z∈0..1000` 且 `500`=中位,也有固件为 `-1000..1000`)。这是 P1 的首要验证项,不臆断。
  - 归一化接口:代码里用 `u ∈ [-1, 1]`,发送前线性映射到上述整型区间。
- **读深度**:优先 `SCALED_PRESSURE2`(外部深度计)或 `VFR_HUD.alt`;单位统一为**米,向下为正深度**(P2 里明确并做符号自检)。
- **连接**:`mavutil.mavlink_connection("udpin:0.0.0.0:14550", dialect="ardupilotmega")`,收到第一包后即可回发指令(与现有 `recorder.py` 相同机制)。

---

## 4. 安全设计(贯穿所有下水阶段)

所有闭环/开环脚本都强制内置以下保护(集中在 `config/vehicle.yaml`):

1. **指令限幅**:第一阶段 `|u| ≤ U_MAX = 0.3`(归一化),确认稳定后再逐步放开。
2. **看门狗 (watchdog)**:主循环若超过 `WATCHDOG_MS = 300 ms` 未发出新指令,自动回中位(`u=0`/z 中位)。
3. **深度软限位**:`DEPTH_MIN / DEPTH_MAX`(如 0.2 m ~ 池深-0.5 m),超限立即中位并退出。
4. **一键急停**:键盘中断 (`Ctrl+C`) 与专用 stop 逻辑都保证发出中位指令后再退出;绝不留下持续指令。
5. **解锁/模式**:脚本启动前打印当前 arm 状态与飞行模式;解锁(arm)必须显式确认,默认不自动解锁。
6. **先干测/系留**:P1 首次通电优先在**系留或出水**状态下,用最小指令观察各推进器方向,再入水。

---

## 5. 核心参数与配置文件

所有可调参数集中在 [`config/vehicle.yaml`](config/vehicle.yaml),代码只读该文件,不硬编码。分组如下:

- **connection**:endpoint、心跳超时、控制频率 `CTRL_HZ`(默认 10 Hz)。
- **safety**:`U_MAX`、`WATCHDOG_MS`、`DEPTH_MIN/MAX`、是否允许自动 arm。
- **manual_control**:z 中位值、各轴正负号(P1 实测后填入)。
- **depth_model**(P3 辨识后填入):一阶 `τ, K` 或二阶 `m_eff, d_lin, d_quad`。
- **pid**(P4):`Kp, Ki, Kd`、积分限幅、输出限幅。
- **mpc**(P5):预测步长 `dt`、预测时域 `N`、状态权重 `Q`、控制权重 `R`、控制增量权重 `R_du`、输入约束、深度约束、求解器。

初始建议值(占位,后续按实测调整):

```yaml
safety:   { U_MAX: 0.3, WATCHDOG_MS: 300, DEPTH_MIN: 0.2, DEPTH_MAX: 3.0, allow_arm: false }
control:  { CTRL_HZ: 10 }
pid:      { Kp: 3.0, Ki: 0.5, Kd: 1.0, i_limit: 0.3, u_limit: 0.3 }
mpc:      { dt: 0.1, N: 20, Q_pos: 10.0, Q_vel: 1.0, R: 0.1, R_du: 0.5, u_limit: 0.3 }
```

---

## 6. 各阶段详细步骤

### Phase 0 — 环境与连接(桌面即可)
- **目标**:venv 建好,`import pymavlink` OK;运行连接自检,收到 heartbeat,打印飞控 system/component、arm 状态、飞行模式,并能持续读到深度消息。
- **步骤**:① 建 venv 装依赖;② 在 BlueOS 配好 14550 endpoint;③ 运行 `python -m src.link --check`。
- **预期结果**:终端打印 `Heartbeat from system=1,...`,深度值随手动升降 ROV 变化。
- **通过标准**:连续 30 s 稳定收到 heartbeat + 深度,无丢包报错。

### Phase 1 — 开环伪手柄驱动(**最低目标达成点**)
- **目标**:代码发归一化 `MANUAL_CONTROL`,推进器按预期方向动;**实测确定 z 中位值与各轴正负号**。
- **步骤**(系留/出水,`U_MAX=0.3`):
  1. 发 `x=+0.2` 1 s → 观察前进推进器;`x=-0.2` → 反向。
  2. 发 `z` 微调,扫出中位(电机停转的值)与上浮/下潜方向。
  3. 每条指令后自动回中位;验证看门狗、Ctrl+C 急停。
- **预期结果**:各轴方向明确,z 中位与符号写入 `config/vehicle.yaml`。
- **通过标准**:能可靠地用一行代码让指定推进器正/反转并安全停止。

### Phase 2 — 状态反馈
- **目标**:10–50 Hz 稳定读取深度,时间对齐,形成 `observation = (depth, depth_rate)`(depth_rate 数值微分 + 低通)。
- **预期结果**:静止时深度噪声量级已知(σ),depth_rate 不发散。
- **通过标准**:观测流可长时间稳定运行,时延 < 一个控制周期。

### Phase 3 — 系统辨识(水池)
- **目标**:采集若干 heave 阶跃(如 `u = ±0.1/±0.2/±0.3`)的深度响应,拟合模型。
  - 一阶(速度模型):`ż = -(1/τ) z + K u`
  - 二阶(推荐):`m_eff·z̈ = K_u·u - d_lin·ż - d_quad·ż|ż|`(含浮力/配重残差项)
- **预期结果**:拟合模型能复现阶跃响应(可用参考论文的 6-DOF 模型作对照/初值)。
- **通过标准**:模型仿真与实测阶跃的 RMSE 在可接受范围;参数写入 `config/depth_model.yaml`。

### Phase 4 — PID 基线(水池)
- **目标**:外环 PID 定深,作为 MPC 的对比基线。
- **基线实验**:定深到目标 `d*`,记录阶跃(如 0.5 m→1.0 m)与定点保持。
- **评价指标(所有控制器统一)**:上升时间、超调量、稳态误差 RMSE、控制能量 `∑u²`、抗扰恢复时间(手动推一下/加恒定拉力)。
- **通过标准**:能稳定定深、稳态误差在传感器噪声量级附近;指标记录进 `data/` 与 `PROGRESS.md`。

### Phase 5 — MPC 闭环(水池)
- **目标**:用 MPC(CasADi)替代 PID,基于 P3 模型在线滚动优化归一化 heave 指令。
- **MPC 形式**:状态 `x=[depth, depth_rate]`,输入 `u∈[-U_MAX,U_MAX]`,目标跟踪 `d*`;
  代价 `Σ Q_pos·(depth-d*)² + Q_vel·ż² + R·u² + R_du·Δu²`;含输入约束与深度软约束;时域 `N`,步长 `dt`。
- **对比实验**:与 P4 相同的阶跃与抗扰工况,用相同指标对比 PID vs MPC。
- **通过标准**:MPC 在超调/抗扰/控制能量至少一项显著优于 PID,且不违反约束。

### Phase 6 — 扩展(可选,需再讨论)
- yaw 单维、水平面(需 DVL 或替代定位)、多 DOF 联合 MPC、或把控制器换成 RL(复用现有 `bluerov2_rl_logger` 的 (obs, action) 采集)。

---

## 7. 预期结果与实验基线汇总

- **最低目标(P1)**:一行代码让指定推进器按归一化指令正/反转并安全停止。✅ 即"让机器人动起来"。
- **基线(P4)**:PID 定深的定量指标(上升时间/超调/RMSE/控制能量/抗扰恢复)。
- **主结果(P5)**:MPC vs PID 在同一工况下的对比表 + 深度-时间曲线图,存于 `data/` 和 `docs/`。

---

## 8. 版本管理与监管记录协议

- **本地 git**:项目已 `git init`(本地)。你在 GitHub 建好空仓库后,用
  `git remote add origin <url>` 关联并 `git push -u origin main`。
- **提交节奏**:每个 Phase 通过验收后做一次提交,message 形如 `P1: open-loop pseudo-stick verified (z neutral=..., signs=...)`。
- **记录文件** [`PROGRESS.md`](PROGRESS.md):每步记录**日期、目标、所用命令/参数、实测结果、遇到的问题、你的决定**。
- **监管**:任何**下水**或**解锁(arm)**动作前,以及每次**进入下一 Phase 前**,我都会先向你说明将做什么并征得同意。
- `.gitignore` 已排除 `.venv/`、`data/`、`__pycache__/` 等。

---

## 9. 参考资料

见 [`docs/references.md`](docs/references.md)。核心三份:
1. `Stationkeep Discussion.docx` — DVL + Position Hold 原理、官方 PSC 参数、单维低速测试建议(本项目选深度的依据)。
2. `MPC_control_for_the_BlueROV2_Theory_and_Implementation.pdf`(AAU 2020 硕士论文)— 6-DOF 建模、Kalman、CasADi/IPOPT NMPC 蓝本。
3. `2506.21063v1.pdf` — arXiv 参考(R4/BlueOS 相关)。
4. [HKPolyU-UAV/bluerov2](https://github.com/HKPolyU-UAV/bluerov2) — 结构参考(本项目做极简 Python 版,不跑其 ROS 环境)。

---

## 10. 当前状态

**已完成(仿真层面)**:
- 项目骨架 + 文档 + 本地 git/GitHub。
- **P0** 连接自检 `src/link.py`(离线用模拟器验证)。
- **SITL** 深度动力学 `src/plant.py` + 闭环 MAVLink 仿真 `tests/sim_vehicle.py`。
- **P1** 伪手柄 `src/pseudo_stick.py` + 安全逻辑严格验证(单元 7/7 + SITL 集成)。
- **P2** 深度状态估计 `src/state.py`(2 阶卡尔曼)+ 噪声验证 `tests/verify_state.py`(全部通过)。
- **P4** PID 定深闭环基线 `src/depth_control.py`+`src/pid.py`+`src/metrics.py`,含控制器侧看门狗;基线指标见 [`docs/RESULTS_P4.md`](docs/RESULTS_P4.md)(阶跃超调~12%、稳态RMSE 3.7cm、抗扰峰值偏移~13cm)。
- **真机准备**:解锁/上锁 + 方向标定 `src/calibrate.py` + 只读 failsafe 查询 `src/check_params.py` + 现场手册 `docs/FIELD_TEST.md`。

- **P5** MPC 定深 `src/mpc.py`(CasADi NMPC:输入硬约束 + offset-free 扰动观测 + 延迟补偿),与 PID 基线对比见 [`docs/RESULTS_P5.md`](docs/RESULTS_P5.md)。结论:1 维定深两者相当(PID 更省力、MPC 超调略小、都 offset-free);MPC 的约束/多 DOF/预测优势待真机+约束场景激发。

- **P3** 系统辨识框架 `src/sysid_collect.py`+`src/sysid_fit.py`(仿真误差最小化,SITL 验证深度回放 R²=0.998),见 [`docs/RESULTS_P3.md`](docs/RESULTS_P3.md);`identified` 段待真机入水数据回填。

**待真机复核**:P0/P1 联机、z 中位与符号、ArduSub 失联行为、深度噪声 σ 标定与到达率、PID/MPC 参数重调、P3 入水采集回填 `identified`。
**后续阶段**:真机 P3 辨识 → 用真实模型重评估 PID/MPC;P6 扩展(多 DOF / yaw / RL)。
