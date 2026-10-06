# BlueROV2 定高前进 — 交接文档

> 最后更新:2026-10-06 | 仓库 `C:\bluerov2_mpc` | GitHub `Yaotseng3tsu/bluerov2_MPC` (main)
> 上一阶段详细流水见 `PROGRESS.md`,现场记录见 `waypoint/RESULTS.md`

---

## 1. 核心目标

纯 Python + pymavlink(**不用 ROS**)控制 BlueROV2,通过 `MANUAL_CONTROL` 伪手柄指令实现:

> **稳定在离底 0.8 m → 向前行进 2 m**

当前架构 = **三路 PID 并行,叠加到同一条指令**:

```
高度 PID(z) ← DVL 离底高度(经 KF 估计)    ┐
距离 PID(x) ← DVL 航位推算 s = ∫vx·dt      ├─► MANUAL_CONTROL(x, 0, z, r) ─► 飞控 ─► 8×T200
航向 PID(r) ← 飞控 ATTITUDE.yaw            ┘
```

阶段机:`HOLD`(稳到目标高度)→ `ADVANCE`(三闭环同时控)→ `DONE`(反推刹车+稳住)

---

## 2. 既定成果

### 2.1 代码清单

| 文件 | 职责 |
|---|---|
| `src/pseudo_stick.py` | **唯一指令出口**:归一化 u → `manual_control_send`;含 `warn_if_rival()` 竞争源检测 |
| `src/pid.py` | PID(积分限幅 + 条件积分抗饱和) |
| `waypoint/dvl_stream.py` | DVL 读取(后台线程、断线自愈、`latest_valid()` 容忍丢帧) |
| `waypoint/altitude_estimator.py` | **2 状态 KF**:新息门控 + vz 惯性桥接 + 跳变再基准 |
| `waypoint/altitude_hold.py` | 定高 |
| `waypoint/go_forward.py` | **主脚本**(三闭环 + 阶段机 + 全部安全逻辑) |
| `waypoint/heading_hold.py` | 航向 PID(含积分;误差 wrap 走最短转向) |
| `waypoint/z_move.py` | z 轴上浮/下潜 + 自动判读 `sign_z` |
| `waypoint/hold_jog.py` | ALT_HOLD 友好点动(不 arm/不换模式/不 disarm) |
| `waypoint/dvl_dashboard.py` | 实时网页仪表盘(纯 stdlib,离线可用,`:8080`) |
| `waypoint/dvl_traj_log.py` / `plot_dvl_traj.py` | 无界面记录 / 离线出图 |
| `waypoint/sim/sim_waypoint.py` | 4DOF SITL + 故障注入 |
| `tests/test_safety.py` | 安全测试(应 **9/9**) |

### 2.2 真机实测参数(2026-09-17 水池)

| 量 | 值 |
|---|---|
| **悬停垂直指令** | `u_z ≈ −0.6`(负浮力,需恒定上推力) |
| 上浮速度 @ `u=0.8` | ≈ 0.085 m/s |
| **前进推力门槛** | `u_x < 0.5` 几乎不动 |
| 前进速度 | `u_x=0.6 → 0.060 m/s`;`0.7 → 0.085 m/s` |
| 定高稳态误差 | ~7 mm |
| DVL 更新率 / 无解率 | 4.5–7.5 Hz / **约 2.5%** |

### 2.3 关键配置(`config/vehicle.yaml`)

| 字段 | 值 | 来源 |
|---|---|---|
| `manual_control.sign_z` | **−1** | 水中实测(原 +1 上下颠倒) |
| `manual_control.sign_x/y/r` | +1 | 已验证(标准 Heavy) |
| `manual_control.z_neutral` | 500 | 已验证 |
| `connection.endpoint` | `udpin:0.0.0.0:14550` | BlueOS 配 UDP Client |

飞控参数(**已改,勿回退**):`JS_GAIN_DEFAULT` 0.2 → **0.8**;`JS_GAIN_MAX` 0.5 → **1.0**

### 2.4 已查出并修复的 7 个真问题

1. **Cockpit 手柄以 25Hz 抢 `MANUAL_CONTROL`**(脚本仅 10Hz)→ ~70% 指令被冲掉,表现为"满推也推不动"。**这是 9/17 后半段所有怪现象的根因**
2. `parse_depth` 未按配置过滤 → `VFR_HUD`(恒 0)与 `GLOBAL_POSITION_INT`(真值)交替喂入,深度测量失效
3. `is_fresh()` 只看最新帧 → 2.5% 坏帧必然误判"丢底锁"中止
4. DVL 高度外点(跳 2.5m)撞 `alt_max` 中止
5. **航向估计跳变 → 控制器满舵 → 机器人真被抡起来**(危险)
6. 设定值跑飞(`v_cruise` 远超实际能力)→ 滞后 1.18m、`u_x` 74% 饱和,PID 退化成开关控制
7. 仿真池底是假的(只钳位 altitude,机器人穿底)→ 矛盾数据喂坏估计器

### 2.5 核心设计决策

- **高度反馈用 DVL 离底高度,不用压力深度**(后者基准漂,且曾被污染)
- **设定值节流**:滞后 > `--max-lag` / `--alt-lag` 时暂停推进设定值,自动适配真实速度
- **速度前馈 `--x-ff`** 跨过推力门槛;到目标后**平滑衰减**(突然归零会反推过头)
- 对齐 ArduSub:控制器吃**估计值**而非原始传感器;glitch 后**重设基准**而非追阶跃
  - 依据:`ArduSub/mode_althold.cpp` 用 `get_pos_estimate_U_m()` 且目标锚在当前估计;
    `ArduCopter/surface_tracking.cpp` glitch 清除后 `init_pos_terrain_D_m(-terrain_u_m)`

### 2.6 两类跳变处理**不同**(别照搬)

| | 处理 | 原因 |
|---|---|---|
| **yaw 参考跳变** | **平移目标**,不打舵 | 物理指向没变,只是参考系变了 |
| **高度基准跳变** | **目标不平移**,只清积分 | 物理距离确实变了,由 slew/节流平滑跟进 |

---

## 3. 约束条件

### 3.1 下水操作规程(必守)

1. **跑脚本前断开 Cockpit/QGC 手柄** —— 否则抢信号,一切现象都不可信
2. `FS_PILOT_TIMEOUT` 放宽到 **30s** —— 手柄断开后脚本停发会 disarm → 负浮力沉底
3. **水下绝不 disarm** —— 退出时交回 `ALT_HOLD` 且保持 armed(脚本默认已如此)

### 3.2 开发约束

- **每步需用户监管确认**;任何解锁/下水前征得同意
- 进度记入 `PROGRESS.md` + git commit,commit 尾注:
  `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`
- git commit 用 `-c user.name="Yaotseng3tsu" -c user.email="zeng@robot.t.u-tokyo.ac.jp"`
- Windows/PowerShell;控制台 cp932 → 脚本内强制 stdout utf-8
- **SITL 不可用于标定参数**:仿真 surge 在 `u=0.6` 终速 1.2 m/s,真机仅 0.06 m/s(**差 20 倍**);
  垂直需 `--net-buoy 48` 才匹配真机悬停推力。SITL **只验证逻辑与故障处理**
- 任何改动后必须:`python tests/test_safety.py`(9/9)+ SITL 回归
- SITL 回归**必须用隔离端口**(14598/14599/16299),**勿碰真机的 14550**
- patch 脚本必须**逐条校验替换是否匹配** —— 曾因未校验导致补丁静默失效

### 3.3 数据可信度(重要)

**9/17 的 `go_forward` 数据全部在手柄冲突污染下采集**,推力门槛、yaw 偏移等结论**都需干净重测**。
唯一干净的是 `go_forward_clean_1m*.csv`(16:01 / 16:17,修复之后)。

**逐文件的可信度判定见 [`docs/packaging/README_DATA.md`](docs/packaging/README_DATA.md)** ——
9/17 五个 bug 各自的发现时刻、每个 CSV 可信/不可信的逐条结论、DVL 2.5% 无解率出自哪个文件。
本节是它的摘要,有疑问以那份为准。

---

## 4. 当前进度

### 4.1 刚完成

**高度估计重构**(对齐 ArduSub)。合成数据 5 种子对照:

| 指标 | 旧 | 新 | 改善 |
|---|---|---|---|
| 全程 RMSE | 0.0943 | **0.0360** | 2.6× |
| 最大误差 | 0.6718 | **0.3050** | 2.2× |
| 速率 RMSE | 0.2466 | **0.0641** | **3.8×** |

SITL 回归:干净末态高度误差 **−0.002m**;注入(外点3% + 丢帧2.5% + 航向尖刺5% + 永久105°平移)
末态 **−0.001m**、外点全被拒、`|u_r|` 峰值 **0.00**。安全测试 **9/9**。已 push。

### 4.2 下一步(按优先级)

**① 干净环境重跑基线**(手柄断开 + 新估计器,这是第一组可信数据):
```powershell
.venv\Scripts\python -m waypoint.dvl_dashboard --tag clean --reset   # 另开窗口, 浏览器 localhost:8080
.venv\Scripts\python -m waypoint.altitude_hold --target 0.8 --u-bias -0.6 --alt-min 0.3 --alt-max 1.2 --label clean_alt
.venv\Scripts\python -m waypoint.go_forward --alt 0.8 --dist 1.0 --max-dist 1.4 --label clean_1m
.venv\Scripts\python -m waypoint.go_forward --alt 0.8 --dist 2.0 --max-dist 2.3 --label clean_2m
```

**② 重新标定推力-速度曲线** → 定 `--x-ff` / `--v-cruise`(现值出自受污染的 f05)

**③ 重新打包交付件** —— 旧 `dist/` 已于 2026-10-06 删除(手工打的包,仓库里没有打包脚本;
其中 60 个 CSV / 2 张图 / 全部代码在 `waypoint/data/` 与仓库里都有原件,已逐项核对)。
重新打包的底稿全在 **`docs/packaging/`**:
- `README_package_full.md`(238 行)/ `README_package_code_only.md`(230 行)—— 两份包 README 原文,
  与仓库 README 是**不同的文档**(CSV 逐列定义 / 四个入口用法 / 下水前三件事),直接拿来改
- `PACKING_MANIFEST.md` —— 两个包的文件清单(没有打包脚本,这是唯一依据)
- `README_DATA.md` —— 数据可信度台账

`dist/` 已加进 `.gitignore`:**交付 zip 本地生成、不入库**。
⚠️ 数据部分必须换成 ① 的干净基线,**不能再用 9/17 那批**(受手柄冲突污染)。

**④ 若航向跳变频繁** → 查罗盘(已知磁力计不可靠),或先 `--no-yaw` 跑通定高前进

### 4.3 已知未决

- 航向估计跳变的**根因**(EKF 重对准 / 罗盘干扰)未查清,目前只做了容错
- surge 系统辨识(W2)从未在干净环境完成
- 两个交付包待重新打包(底稿见 `docs/packaging/`,旧 `dist/` 已删;须等 ① 的干净基线出来)

---

## 附:常用命令

```powershell
# 只读自检
.venv\Scripts\python -m waypoint.dvl_stream --seconds 12       # DVL
.venv\Scripts\python -m waypoint.yaw_monitor --seconds 40      # 航向(手动转动核对)
.venv\Scripts\python -m src.link --check --seconds 8           # 链路+深度源

# SITL 回归(隔离端口)
.venv\Scripts\python -m waypoint.sim.sim_waypoint --bind-port 14599 --ctrl-addr 127.0.0.1:14598 `
    --dvl-port 16299 --seconds 100 --z0 1.2 --bottom 2.5 --net-buoy 48 `
    --alt-glitch 0.03 --dvl-dropout 0.025 --yaw-glitch 0.05
.venv\Scripts\python -m waypoint.go_forward --endpoint udpin:0.0.0.0:14598 `
    --dvl-ip 127.0.0.1 --dvl-port 16299 --alt 0.8 --dist 2.0 --u-bias -0.6 `
    --alt-min 0.2 --alt-max 2.0 --yes --label sitl_check

.venv\Scripts\python tests\test_safety.py                      # 应 9/9
```
