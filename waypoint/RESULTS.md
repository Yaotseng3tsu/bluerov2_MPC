# Waypoint 相对航点定向航行 — 实测记录与复盘

> W6 现场记录模板。连机/下水当天逐项填写；`[ ]` 勾选，`___` 填值，⬜ 段落待补。
> 约定：surge 前向 +X；深度向下为正；航向用 ATTITUDE.yaw；距离 = DVL ∫vx·dt。

- **日期**：2026-09-16
- **地点/水池**：__________（可用直线距离 ≈ ___ m，水深 ≈ ___ m）
- **上位机 IP**：192.168.2.188  **DVL IP**：192.168.2.95:16171
- **在场/监管**：__________
- **软件版本**：commit `a96903c`

---

## 0. 下水前检查（干测/岸上）
- [x] venv 可跑；软件版本 commit `a96903c`
- [x] MAVLink 心跳 OK（飞控 sys1/comp1，autopilot=3，DISARMED/MANUAL）；深度源 GLOBAL_POSITION_INT @26Hz，噪声±5mm
- [x] 航向传感器 OK（`yaw_monitor` 手转验证：ATTITUDE.yaw 跟随全圈，范围[-180,+176]°跨度356°，@20Hz；roll/pitch 实时响应）→ heading_hold 输入可靠
- [x] failsafe 已配：FS_PILOT_INPUT=2 / FS_PILOT_TIMEOUT=3 / FS_GCS_ENABLE=2 / FS_LEAK_ENABLE=1
- [x] `go_waypoint` 全链路(不解锁)真机联调：连接/预热(深度+航向)/DESCEND→TURN 转换/从真实yaw算r指令(ur=+0.30饱和)/CSV/超时安全退出 均正常
- [x] 解锁短点动扫描(diag_motor --scan --u 0.6 --umax 0.6)：SERVO 增量 x[48×4]/y/z/r 全部符合标准 Heavy，与 Day1 一致 → go_waypoint 指令通路能驱动正确电机
- [x] 现场：解锁/退出自动上锁正常
- [x] `go_waypoint --arm` 端到端(动桨)：解锁→DESCEND→TURN(偏航桨转,ur=+0.50)→超时停+自动上锁，完整任务脚本真机驱动电机验证通过
- [ ] 电量/漏水/系缆检查；桨叶周围清空；急停/断电预案确认

> ⚠️ **推力权限偏低**：u=0.6→±48µs（满程±400 的~12%），u=1.0 推算~±80µs（~20%），ArduSub pilot gain 偏低。
> **明天下水前建议**：BlueOS/Cockpit 调高 pilot gain；或 W2/W5 用 `--umax 0.8~1.0`。否则 surge/yaw 偏弱偏慢（首测偏安全，但 W2 需能激起可测速度）。

---

## 1. W1 — DVL 接入验证
命令：`python -m waypoint.dvl_stream --seconds 12`

| 项 | 期望 | 实测 | 判定 |
|---|---|---|---|
| 直连 16171 | 能连（扩展未停用） | ✅ 直连成功（扩展未挡） | [x] |
| 更新率 | ≈ 8–15 Hz | ≈4.4 Hz（气中/水中一致，A50 report rate） | [x] |
| 底锁 valid | 贴底 True，altitude>0 | ✅ 水中：alt=1.49m，fom=0.0014，valid=True | [x] |
| **vx 前进符号** | 前进为正 | ___（需底锁） | 待下水 |

- **vx 符号结论**：前进为正 → `--vx-sign 1`；前进为负 → **`--vx-sign -1`**（记此处，勿改固件）。⬜ 待下水
- 直连 16171 成功，**无需停用 BlueOS DVL 扩展**（走直连路线）。
- 气中更新率 4.7Hz 偏低；A50 无底锁常降速，**下水贴底后复测**（若仍~5Hz，10Hz 环用零阶保持够用）。
- 备注：⬜

---

## 2. W2 — surge 系统辨识
采集：`python -m waypoint.surge_sysid_collect --umax ___ --max-dist ___ --vx-sign ___ --arm --yes --label pool1`
拟合：`python -m waypoint.surge_sysid_fit waypoint/data/surge_sysid_pool1.csv`（确认图后加 `--write`）

**采集条件**：档位 `_____________`  每档保持 ___ s  回中 ___ s  距离护栏 ±___ m
- [ ] 各档推进器确实转动（顶过 ESC 死区）；死区实测：u≈___ 以下不转
- [ ] 深度维持稳定（操作者/或先手动定深）；无撞墙（护栏触发 ___ 次）

**拟合结果**（`eff_mass` 固定 = ___）：

| 参数 | 值 | 备注 |
|---|---|---|
| K_thrust_N | ___ | u=1 前向推力 |
| c_lin | ___ | 线性阻尼 |
| c_quad | ___ | 二次阻尼 |
| b0（残余加速度） | ___ | 推力不对称/残余水流 |
| R² / RMSE | ___ / ___ m/s | 拟合优度 |
| u=0.5 稳态终速 | ___ m/s | 选巡航速度参考 |

- [ ] 拟合图（`*_fit.png`）测量 vx 与模型贴合良好
- [ ] `--write` 已回填 `config/surge_model.yaml → identified`
- **对占位 sim_truth 的偏差**（K50/c_lin4/c_quad18）：⬜
- 备注：⬜

---

## 3. W5 — 定向航行实测
命令模板：
```
python -m waypoint.go_waypoint --heading <deg> --dist <m> --depth <m> \
    --v-cruise <m/s> --umax <val> --deadzone <val> --vx-sign <±1> --arm --yes --label <tag>
```
> 巡航速度参考 W2 终速；`--umax/--deadzone` 顶过 ESC 死区（据 W2 死区实测调）。
> 先小距离（1 m）短测，逐步加到 3 m。CSV 存 `waypoint/data/go_waypoint_<tag>.csv`。

| # | heading° | dist目标 | depth目标 | v_cruise | 实到 s(DVL) | 末深 | 末航向 | reason | 目测偏差 | 备注 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | | 1.0 | | | | | | | | 短测 |
| 2 | | 3.0 | | | | | | | | |
| 3 | | 3.0 | | | | | | | | 换航向 |

- **DVL 距离 vs 目测/卷尺真实距离**（评估 DVL 标定误差）：DVL=___ m，实测=___ m，误差 ___%
- **航向保持**：直线偏移目测 ___ m / 末航向误差 ___°
- **深度保持**：巡航中深度波动 ±___ m
- **安全降级实测**（可选）：DVL 遮挡→是否 DVL_LOST 停？⬜；倾斜/超深→是否停？⬜

---

## 4. 问题与调整
| 现象 | 原因分析 | 处理 | 结果 |
|---|---|---|---|
| | | | |

---

## 5. 结论与下一步
- **达成**：⬜（是否实现"转向→前进指定距离→到深度"？精度如何？）
- **关键数据沉淀**：surge `identified` 参数、vx 符号、ESC 死区/gain、DVL 标定误差量级。
- **下一步候选**：① 提高 pilot gain / U_MAX 权限；② sway 辨识→holonomic 全向；
  ③ surge/yaw 换 MPC（约束/预测）；④ 闭环位置（多次往返或外部定位）。
- 备注：⬜

---
*填写后 `git add waypoint/RESULTS.md && git commit`；数据 CSV/PNG 默认 gitignore，可另行归档。*
