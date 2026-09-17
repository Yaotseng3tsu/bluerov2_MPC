# Waypoint — 相对航点定向航行 (Point-and-go 4DOF)

> 目标：在**无外部绝对定位**下，用伪手柄让 BlueROV2 执行一次相对航点移动：
> **转到目标航向 → 前进指定距离 → 调到目标深度**，全程保持航向与深度。
> 例：`go_waypoint --heading 90 --dist 3 --depth 1.5`（朝正东走 3m、深度 1.5m）。
>
> 方案 = **surge 动力学前馈** + **DVL 速度航位推算 (∫v·dt) 判到距** +
> **IMU/ATTITUDE 航向保持** + **绝对深度闭环**（复用主项目深度模型）。

本文件夹是 `C:\bluerov2_mpc` 主项目子模块，复用
[`src/pseudo_stick.py`](../src/pseudo_stick.py)、[`src/plant.py`](../src/plant.py)、
[`src/depth_control.py`](../src/depth_control.py)、`sysid` 范式与安全约定。

---

## 0. 能控自由度与数据现状（重要）

**BlueROV2(Heavy)只独立驱动 4 个自由度**：`surge(x) / sway(y) / heave(z) / yaw(r)`。
**roll、pitch 无独立执行机构**，靠重浮心分离被动稳定 → 本模块只**监测**其角度（超限报警），
不作为航行目标。所谓"6 自由度定向航行"这台机器物理上做不到；4DOF 相对航点已覆盖实用需求。

| 自由度 | 目标量 | 闭环来源 | 数据现状 |
|---|---|---|---|
| surge x | 前进距离 | DVL `vx` 航位推算 | 需 W2 辨识 |
| sway y | 侧移距离 | DVL `vy` 航位推算 | **本期预留接口**，暂不辨识 |
| heave z | 目标深度 | **绝对深度** GLOBAL_POSITION_INT | ✅ 已有模型+KF，真闭环最准 |
| yaw r | 目标航向 | IMU/ATTITUDE `yaw` | ✅ 现成（不用磁力计） |

**为何要补 surge 辨识**：深度模型只标定 heave（水平轴分配/附加质量/阻尼全不同，不能挪用）；
`rl_logger` 拖曳 IMU 24 组是被动拖曳、无遥杆指令通道，只含姿态/起停/方向可分性，无"指令→运动"映射。
⇒ 必须补一小段 surge 辨识（W2）；有 DVL 速度反馈后成本很低。

---

## 1. 范式与坐标约定

- **范式：Point-and-go 4DOF**（本期）——分解为①调深度 ②转航向 ③沿航向前进到距离。
  非全向：不同时用 sway 斜move。**sway 接口预留**，后续可升级为 holonomic 全向。
- **surge**：机体前向 +X，指令 `u_x∈[-1,1]`（复用 `manual_control.sign_x`）。
- **DVL 速度**：`vx/vy` 为机体系速度；W1 连机确认 `vx` 前进符号。
- **坐标系**：航位推算在**世界系**做——用 `yaw` 把机体系 DVL 速度旋到世界系再积分
  （`go_waypoint` 里 `[vN;vE] = R(yaw)·[vx;vy]`），这样航向变化不污染距离。
- **航向**：ArduSub 融合 `ATTITUDE.yaw`，**不用磁力计**（不可靠）。
- **深度**：绝对深度闭环（向下为正），复用主项目 depth 模型/PID。
- **距离判据**：沿目标航向的位移 `s=∫v_along·dt ≥ dist` 即停（相对位移，非绝对位置真值）。

---

## 2. 目标架构

```
 航点(heading*, dist, depth*)
        │
        ▼
 ┌──────────────┐
 │  go_waypoint │  ①depth闭环  ②yaw转向  ③surge前馈+航位推算   ← 顺序/可叠加
 │  (状态机)     │
 └──┬───┬───┬───┘
    │   │   └── surge: motion_model 前馈 u_x  ─┐
    │   └────── yaw:   heading_hold → r        ├─▶ pseudo_stick(x,z,r 安全发送)
    │  depth: depth_control(复用) → u_z ───────┘
    ▼
 ┌──────────────┐  vx,vy (DVL) ─ R(yaw) ─▶ 世界系位移 s   深度(MAVLink)  yaw(ATTITUDE)
 │ 航位推算+停止 │  底锁丢失/超龄 → 立即安全停
 └──────────────┘
```

---

## 3. 文件规划（随步骤逐个创建，均先经你确认）

| 文件 | 作用 | 步骤 | 状态 |
|---|---|---|---|
| `README.md` | 本文档 | — | ✅ |
| `dvl_stream.py` | DVL 速度读取器（线程安全 latest/is_fresh/超龄） | W1 | ✅ |
| `config/surge_model.yaml` | surge 二阶模型（`sim_truth`+`identified`；预留 sway 段） | W2 | ✅ (identified 待真机) |
| `surge_sysid_collect.py` / `surge_sysid_fit.py` | 发 x 阶跃、DVL 记 vx → 拟合回填 | W2 | ✅ 代码 (待采集) |
| `motion_model.py` | surge 1D 动力学 + 梯形速度轨迹（给距离→前馈 u_x(t)） | W3 | ✅ |
| `heading_hold.py` | ATTITUDE.yaw 航向 P/PD 控制 → r 修正 | W4 | ✅ |
| `go_waypoint.py` | **主脚本**：4DOF 状态机 + 世界系航位推算 + 安全降级 | W5 | ✅ (SITL) |
| `sim/fake_dvl.py` + `sim/sim_waypoint.py` | 离线仿真件（假 DVL + 4DOF SITL） | 仿真 | ✅ |
| `RESULTS.md` | 干测/水下记录与复盘（现场填写模板） | W6 | ✅ 模板 (待填) |

深度沿用主项目 [`src/depth_control.py`](../src/depth_control.py)+[`src/plant.py`](../src/plant.py)，不新建。

### DVL 观测工具（移植自 `rl_logger`，实机水中验证过）
与控制无关、纯观测/记录，**可与 go_waypoint/sysid 同时连 DVL**（已验证 A50 支持多客户端）：

| 文件 | 用途 | 依赖 |
|---|---|---|
| `dvl_dashboard.py` | 后台连 DVL→写 CSV + 本地实时网页仪表盘（XY 轨迹/速度/高度/yaw/底锁） | 无（stdlib） |
| `dvl_traj_log.py` | 无界面：记 velocity + position_local 两路 CSV（`--reset` 归零 DR） | 无 |
| `plot_dvl_traj.py` | 离线出图（含 velocity 世界系积分对照） | matplotlib/numpy |

CSV 存 `waypoint/data/dvl_data/`，字段与 `rl_logger` 完全一致（两项目数据互通）。
控制环仍用 [`dvl_stream.py`](dvl_stream.py)（velocity-only、线程安全）——两者读同一 16171 流、各开 socket、互不影响。
用法：`python waypoint/dvl_dashboard.py --tag <标签> --reset` → 浏览器开 `http://localhost:8080`。
> 注：`position_local` 的 yaw 无罗盘校正会漂移（~33°/44s）；采正式轨迹前在 BlueOS 扩展打开 `Enable DVL driver` 让飞控航向校正 yaw。短程（3m/~10s）漂移可忽略。

---

## 4. 分步骤计划（离线优先；每步执行前先征求你的意见）

> 规则：**每步先跟你确认方案/参数 → 我实现 → （能离线的先仿真验证）→ 记录 → 再进下一步。**
> 连机 / 解锁 / 下水前必须显式同意。进度记入主项目 `PROGRESS.md` 并 commit。
>
> **离线可做**：W3 全离线；W4、W5 逻辑靠"假 DVL + surge/yaw SITL"验证。
> **必须连机**：W1 真验证、W2 真实采集、W6 实测。

### W1 — DVL 接入自检 ✅（代码完成，连机验证待明天）
保留 BlueOS DVL 扩展；`dvl_stream.py` 优先直连 16171 只读，占用则退回 MAVLink 读转发速度。
明天连机确认：能否直连、更新率、`vx` 前进符号（见 `--forward-hint`）。

### W3 — 运动模型 + 前馈轨迹（**现在做，全离线**）
- `motion_model.py`：surge 一维 RK4 动力学（照搬 `src/plant.py`）+ 梯形速度轨迹生成器
  （加速—匀速—减速，末端零速；给定 `dist` 与 `v_cruise/a_max` → `u_x(t)` 前馈序列）。
  轨迹生成写成 **DOF 无关**（同一梯形逻辑将来可用于 sway）。
- **产出/判据**：离线仿真里前馈积分位移≈目标距离、末端速度≈0；出图给你看。
- **需你确认**：巡航速度上限 `v_cruise`、加速度 `a_max`、末端减速余量。
- **注**：辨识前先用 `surge_model.yaml` 的 `sim_truth` 占位跑通，W2 后换 `identified`。

### W4 — 航向保持（离线 SITL 验证）
- `heading_hold.py`：锁定目标 `yaw`，P（必要时 PD）输出 `r`；`r` 限幅、角度 wrap 到 ±180°。
- **产出/判据**：SITL 注入 yaw 扰动能收敛回设定航向。
- **需你确认**：航向增益、`r` 限幅、到位容差。

### W2 — surge 系统辨识（需连机采集；代码可离线先写）
- 档位**顶过 ESC 死区**：`u_x=0.35/0.45/0.55`（前后各点动），采集须 `--umax 0.6` 覆盖 `U_MAX=0.3`。
- `surge_sysid_collect.py` 发档位 + DVL 记 vx → CSV；`surge_sysid_fit.py` 仿真误差最小化拟合
  `(m_eff,c_lin,c_quad,K_x)`，`--write` 回填 `config/surge_model.yaml → identified`。
- **需你确认**：每档时长 / 水池可用直线距离（避免撞墙）；是否往返取平均。
- **注**：采集/拟合代码可离线对 surge SITL 先验证；真实参数必须连机采。

### W5 — 主脚本 go_waypoint（逻辑离线；末端精度调参需连机）
- `go_waypoint.py --heading θ --dist d --depth z`：状态机
  ①深度闭环到 `z`（复用 depth_control）②yaw 转到 `θ`（heading_hold）③按 motion_model 前馈发 `u_x`、
  叠加 heading_hold 的 `r`、depth 的 `u_z`；世界系 `∫v_along·dt` 到 `d` 减速停。
  全程复用 `pseudo_stick` 看门狗/回中位/退出上锁；DVL 超龄或丢底锁 → 立即安全停。
  预留 `--dy/sway` 参数（本期报未实现）。
- **产出/判据**：假 DVL+SITL 跑通整套逻辑与安全降级；到距/末速/航向/深度误差达标。
- **需你确认**：三阶段是顺序还是深度与前进叠加、停止判据、DVL 失效降级策略。

### W6 — 干测 → 水下实测 → 复盘
先干测（不解锁/短点动验证指令与安全），再水下实测，写 `RESULTS.md`。下水每次解锁前逐条确认。

---

## 5. 安全（继承主项目约定）
- 默认 `allow_arm=false`；解锁/下水前必须显式确认。
- keepalive 线程发指令+GCS 心跳；退出/异常/Ctrl+C → 先回中位，本进程解锁过则自动上锁。
- failsafe（`FS_PILOT_INPUT/TIMEOUT/GCS`）作硬兜底。
- DVL 超龄/丢底锁、深度超龄 → 立即停车回中位。
- roll/pitch 超阈值报警。保守 `U_MAX`；辨识/首测用短行程、留撞墙余量。

---

## 附：后续可扩展
- **Holonomic 全向**：补 sway 辨识 → `go_waypoint --dx --dy` 直接斜move（接口已预留）。
- **MPC**：surge/yaw 前馈换成主项目 MPC 范式（约束/多DOF/预测优势）。
