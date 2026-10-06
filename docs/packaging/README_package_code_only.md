# BlueROV2 — DVL 读取 / 可视化 / 定高 / 定高前进(**纯代码版**)

从 `bluerov2_mpc` 项目中剥出的**最小自包含子集**:读 DVL → 实时可视化 → 用 PID 维持离底高度 →
升到目标高度后向前行进指定距离(默认 2m)。只含这条链路必需的代码,无其它依赖。

> 本包内所有参数默认值均来自 **2026-09-17 真机水池实测**,不是推测值。

---

## 1. 目录结构

```
bluerov2_dvl_altitude_forward/
├── README.md                    本文档
├── requirements.txt             依赖
├── config/
│   └── vehicle.yaml             车辆配置(连接/安全限幅/轴符号/PID)
├── src/                         通用底层
│   ├── pseudo_stick.py          ★ 唯一的指令出口:归一化 u → MAVLink MANUAL_CONTROL
│   └── pid.py                   PID(含积分限幅 + 条件积分抗饱和)
└── waypoint/
    ├── dvl_stream.py            ★ DVL 读取器(后台线程/丢帧容错/跳变剔除)
    ├── heading_hold.py          航向 PID(误差 wrap 走最短转向)
    ├── altitude_hold.py         ★ 定高(高度 PID)
    ├── go_forward.py            ★ 定高前进(高度+距离+航向 三闭环)
    ├── dvl_dashboard.py         ★ 实时网页仪表盘 + 同步记录 CSV
    ├── dvl_traj_log.py          无界面记录(velocity + position_local 两路 CSV)
    ├── plot_dvl_traj.py         离线出图(轨迹/速度/高度/yaw)
    └── data/                    ← **新**运行的产物写这里(出厂为空)
        └── dvl_data/            ← 新的 DVL 记录
```

`★` = 四个主要入口。

---

## 2. 安装与运行

```bash
pip install -r requirements.txt
# 在**包根目录**运行(脚本用 `python -m` 方式导入 src/ 与 waypoint/)
python -m waypoint.dvl_stream --seconds 10
```

### ⚠️ 下水前必须确认的三件事

| 项 | 要求 | 后果 |
|---|---|---|
| **断开 Cockpit/QGC 的手柄** | 跑脚本时不能有第二路 `MANUAL_CONTROL` | 飞控只认最后到达的一条。手柄以 25Hz 刷中位、脚本仅 10Hz → **约 70% 指令被冲掉,满推也推不动**。脚本已内置检测,解锁前会告警 |
| **`JS_GAIN_DEFAULT` / `JS_GAIN_MAX`** | 建议 0.8 / 1.0 | 出厂 0.2/0.5(顶格仅 50%)时推力严重不足,深度与水平都推不动 |
| **`FS_PILOT_TIMEOUT`** | 建议放宽到 30s | 手柄断开后脚本一停发指令就无人喂,超时 disarm → 负浮力机器人沉底 |

---

## 3. 四个入口的用法

### 3.1 `dvl_stream` — DVL 读取自检
```bash
python -m waypoint.dvl_stream --seconds 12          # 流式打印 + 报更新率
python -m waypoint.dvl_stream --forward-hint        # 说明如何确认 vx 前进符号
```
气中 `valid=false` 属正常(声波无反射),**下水贴底**才有解。实测更新率约 4.5–7.5 Hz。

### 3.2 `dvl_dashboard` — 实时可视化(主力)
```bash
python waypoint/dvl_dashboard.py --tag live1 --reset
# 浏览器打开 http://localhost:8080
```
纯 stdlib + 原生 JS,**零 CDN、离线可用**。一边写两路 CSV,一边提供网页:XY 轨迹 / body 速度 /
离底高度 / yaw / 底锁状态。`--reset` 开始前把 DVL 内部航位推算归零。

**可与控制脚本同时运行**(已验证 A50 支持多客户端),建议全程开着做观测。

### 3.3 `altitude_hold` — 定高
```bash
python -m waypoint.altitude_hold --target 0.8 --seconds 30 --u-bias -0.6 \
       --alt-min 0.3 --alt-max 1.2 --label hold1
```
反馈用 **DVL 离底高度**(不是压力深度)。实测稳态误差 ~7mm。

### 3.4 `go_forward` — 升到 0.8m 再前进 2m(本包的主目标)
```bash
python -m waypoint.go_forward --alt 0.8 --dist 2.0 --max-dist 2.3 --label run1
```
阶段:`HOLD`(稳到目标高度)→ `ADVANCE`(高度+距离+航向同时控)→ `DONE`(反推刹车并稳住位置)。

常用调整:
```bash
--no-yaw                 # 航向估计不可靠时关掉航向控制(罗盘受干扰时推荐)
--v-cruise 0.10          # 前进快一点(别超过机器人实际能力,内部有节流自动兜底)
--u-bias -0.65           # 起浮慢就更负一点(实测悬停约 -0.6)
--hold-timeout 60        # 从池底起浮较慢时放宽
```

### 3.5 离线出图
```bash
python waypoint/plot_dvl_traj.py --tag live1     # 自动找最新一对 CSV
```

---

## 4. 数据存储结构

本版**不含数据**(纯代码)。所有运行产物写在 `waypoint/data/`,不会写到包外。
如需随包的历史实测数据,见另一份 `bluerov2_dvl_altitude_forward.zip`。

```
waypoint/data/
├── go_forward_<label>.csv          go_forward 每次运行的完整控制记录
├── altitude_hold_<label>.csv       altitude_hold 的控制记录
└── dvl_data/
    ├── dvl_vel_<tag>_<YYYYmmdd_HHMMSS>.csv    DVL 速度流(原始)
    ├── dvl_pos_<tag>_<YYYYmmdd_HHMMSS>.csv    DVL 航位推算位置(原始)
    └── plot_<tag>_<...>.png                   plot_dvl_traj 出的图
```

### 4.1 `go_forward_<label>.csv`(12 列)

| 列 | 含义 | 单位 |
|---|---|---|
| `t` | 自解锁起的时间 | s |
| `phase` | 阶段:`HOLD` / `ADVANCE` / `DONE` | — |
| `sp_alt` | 高度**设定值**(斜坡+节流后的,非最终目标) | m |
| `alt` | DVL 实测离底高度 | m |
| `u_z` | 发给飞控的垂直指令(**负=上推**) | −1~1 |
| `sp_dist` | 距离设定值(匀速斜坡+节流后) | m |
| `s` | 已走距离 = ∫vx·dt(航位推算) | m |
| `vx` | DVL 实测机体前向速度(已乘 `--vx-sign`) | m/s |
| `u_x` | 发给飞控的前进指令 | −1~1 |
| `yaw` | 飞控 ATTITUDE 航向(跳变已剔除) | ° |
| `yaw_err` | 相对锁定目标的航向误差(wrap 到 ±180) | ° |
| `u_r` | 发给飞控的偏航指令 | −1~1 |

> 判读要点:`sp_alt`/`sp_dist` 与 `alt`/`s` 的差就是**跟踪滞后**。内部节流保证滞后不超过
> `--alt-lag`(0.20m)/`--max-lag`(0.25m);若看到滞后远超这个值,说明节流没生效或机器人被卡住。

### 4.2 `altitude_hold_<label>.csv`(7 列)

| 列 | 含义 |
|---|---|
| `t` | 时间(s) |
| `target` | 最终目标高度(m,恒定) |
| `sp` | 斜坡中的当前设定值(m) |
| `alt` | DVL 实测高度(m) |
| `alt_rate` | 高度变化率(m/s,数值微分+低通) |
| `u_pid` | PID 输出(不含前馈) |
| `u_z` | 实际发出的指令 = `u_bias + u_pid` 再限幅 |

> `u_z` 的稳态值就是**悬停所需推力**,实测约 **−0.6**(负浮力机器人需要恒定上推力)。

### 4.3 `dvl_vel_*.csv`(17 列,DVL 原始速度流)

```
wall_time, time_of_validity, time_of_transmission, time,
velocity_valid, vx, vy, vz, altitude, fom, status,
cov_xx, cov_xy, cov_xz, cov_yy, cov_yz, cov_zz
```
- `wall_time`:PC 墙钟(epoch 秒)—— **跨传感器对齐用**
- `time_of_validity` / `time_of_transmission`:DVL 绝对时间戳(unix **微秒**)—— 最精确,积分轨迹用它
- `time`:报文**间隔**(ms),不是时钟
- `vx/vy/vz`:**body/DVL 系**速度(m/s)。世界系需按 yaw 旋转
- `altitude`:离底高度(m);`fom`:figure of merit(越小越可信)
- `velocity_valid`:底锁有效性。实测约 **2.5% 的帧无解**(`valid=false`/`altitude=-1`),属正常
- `cov_*`:3×3 速度协方差上三角 6 项(供 EKF/加权)

### 4.4 `dvl_pos_*.csv`(10 列,DVL 内部航位推算)

```
wall_time, ts, x, y, z, roll, pitch, yaw, std, status
```
- `x/y/z`:局部系位置(m),`--reset` 后从 0 起算
- `yaw`:**无罗盘校正时会漂移**(实测约 33°/44s)→ 长轨迹会整体旋转。正式采集前应在
  BlueOS 扩展里打开 `Enable DVL driver`,让飞控航向校正 DVL 的 yaw
- `std`:位置标准差

> CSV 字段与原 `bluerov2_rl_logger` 项目**完全一致**,两边数据可互通。

---

## 5. 配置文件 `config/vehicle.yaml`

控制脚本启动时读取。关键项:

| 字段 | 当前值 | 说明 |
|---|---|---|
| `connection.endpoint` | `udpin:0.0.0.0:14550` | BlueOS 需配一个指向本机 14550 的 UDP Client |
| `manual_control.sign_z` | **−1** | **实测值**:原 +1 时上下颠倒。改动前务必用实测验证 |
| `manual_control.sign_x/y/r` | +1 | 已验证(标准 Heavy 分配矩阵) |
| `manual_control.z_neutral` | 500 | MANUAL_CONTROL 的 z 通道中位 |
| `safety.U_MAX` | 0.3 | 默认限幅;脚本可用 `--umax` 覆盖(定高/前进默认用 1.0) |
| `control.CTRL_HZ` | 10 | 控制环频率 |

> `depth_model` / `pid` / `mpc` 等段落属于原项目的**压力深度**控制链路,本包的定高走 DVL 高度,
> **不使用**它们,保留只为配置文件完整。

---

## 6. 内置安全机制(都在代码里,无需额外配置)

| 机制 | 行为 |
|---|---|
| 竞争指令源检测 | 解锁前检测是否有第二路 `MANUAL_CONTROL`,有则告警并要求确认 |
| DVL 丢帧容错 | 判据是"最近一次**有效**帧的龄期",单帧坏数据不会误判丢底锁 |
| DVL 高度跳变剔除 | 变化率超 1.5 m/s 的高度外点被丢弃(实测偶发跳到 2.5m,会误触发护栏) |
| 航向跳变剔除 | 用陀螺 `yawspeed` 交叉校验;持续偏离判为**参考系平移**→ 平移目标而非打舵去追 |
| 航向失效兜底 | 航向持续跳变 >1.5s → 停用航向控制(`u_r=0`),不拿坏估计驱动推进器 |
| 高度护栏 | `alt_min`(防撞底,"先升到安全高度才武装",允许从池底起浮)/ `alt_max`(防冲出水面) |
| 距离护栏 | `--max-dist` 防撞池壁;另有 vx 符号自检(反向位移 >0.3m 即停) |
| 退出保护 | 回中位 → 交回 `ALT_HOLD` → **默认不 disarm**(负浮力机器人避免沉底) |

---

## 7. 实测参考数据(2026-09-17 水池)

| 量 | 实测值 |
|---|---|
| 悬停所需垂直指令 | `u_z ≈ −0.6`(负浮力) |
| 上浮速度 @ `u=0.8` | ≈ 0.085 m/s |
| 前进速度 | `u_x=0.6 → 0.060 m/s`;`u_x=0.7 → 0.085 m/s` |
| 前进推力门槛 | `u_x < 0.5` 几乎不动 → 故用 `--x-ff` 前馈直接跨过 |
| 定高稳态误差 | ~7 mm |
| DVL 更新率 / 无解率 | 4.5–7.5 Hz / 约 2.5% |

---

## 8. 不在本包内(原项目里的其它东西)

压力深度闭环(`depth_control` / `state` / `link`)、surge 系统辨识、MPC、SITL 仿真件、
`go_waypoint`(基于压力深度的 4DOF 航点)等均未收录 —— 本包只保留
"**读 DVL → 可视化 → 定高 → 定高前进**"这一条链路。
