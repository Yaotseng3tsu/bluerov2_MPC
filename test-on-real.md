# 真机实测报告 — Day 1(2026-09-11)

BlueROV2 R4 首次真机测试。**全程出水台架干测(不下水)**,目标 P0(连接)→ P1(代码驱动推进器)。
结论:**P0 + P1 全部完成,最低目标"用代码给归一化指令让机器人动起来"达成。**

---

## 1. 环境与连接

| 项 | 值 |
|---|---|
| 载具 | BlueROV2 R4(Heavy 配置,8× T200) |
| 环境 | 出水/台架干测 |
| 上位机 IP | 192.168.2.188(以太网,BlueROV 网段) |
| 飞控 | ping 192.168.2.2 通;autopilot=3(ArduSub)、type=12(潜航器) |
| 连接方式 | BlueOS(Pirate Mode → MAVLink Endpoints)新建 **UDP Client → 192.168.2.188:14550**;上位机脚本用 `udpin:0.0.0.0:14550` |

**连接踩的坑**
- 初次无 heartbeat = BlueOS 未配指向本机的端点 → 加 UDP Client 端点后解决。
- Norton 防火墙需临时关闭/放行 python 收 UDP 14550。

---

## 2. 各步骤结果

### ① check_params — 只读查 failsafe ✅
| 参数 | 值 | 含义 |
|---|---|---|
| FS_PILOT_INPUT | 2 | 手柄/MANUAL_CONTROL 失联 → disarm |
| FS_PILOT_TIMEOUT | 3 s | 失联判定超时 |
| FS_GCS_ENABLE | 2 | 地面站失联 → disarm |
| FS_LEAK_ENABLE | 1 | 漏水保护开 |
| FS_CRASH_CHECK / FS_EKF_ACTION | 0 / 0 | 干测无影响 |
| BATT_LOW_VOLT | 12 | 低压阈值 |

**结论**:硬崩溃兜底充分——进程崩溃/断网时 ArduSub 约 3s 内自动 disarm;加上代码正常退出/Ctrl+C 主动上锁 = 双保险。

### ② P0 连接自检(link --check)✅
- heartbeat OK,arm=DISARMED,模式=MANUAL。
- 深度源 = **GLOBAL_POSITION_INT**,实测 **11.0 Hz**(heartbeat 3.0 Hz)。
- 深度静态噪声 ≈ **±5 mm**(比仿真假设 20 mm 更干净)。
- 干测深度不随动作变化,属正常;深度符号验证留待下水。

### ③ 未解锁不动 ✅
- 发 z=0.2(不 `--arm`)→ T200 全程不转。安全默认(未解锁指令被忽略)验证通过。

### ④ 轴映射 / 中位标定 ✅（用 SERVO_OUTPUT_RAW 判读)
肉眼难判方向,改用 `diag_motor --scan` 读飞控实际输出 PWM。u=0.6 时各轴 SERVO 相对 1500 的增量(通道 1–8):

```
x=+0.6: [+48, +48, +48, +48,   0,   0,   0,   0]   水平同向 → surge
y=+0.6: [-48, +48, +48, -48,   0,   0,   0,   0]   水平差动 → sway
z=+0.6: [  0,   0,   0,   0, +48, -48, -48, +48]   垂直差动 → heave(Heavy 标准)
r=+0.6: [-48, +48, -48, +48,   0,   0,   0,   0]   水平力偶 → yaw
```

**结论**:标准 BlueROV2 Heavy 推力分配矩阵,四轴映射全部正确且独立。
- 电机 1–4 = 水平(矢量);5–8 = 垂直。
- `sign_x/y/z/r = +1`,`z_neutral = 500`(中位→SERVO 1500,已验证)。
- **绝对方向符号(尤其 z+ 是否 = 下潜)干测无法确定,待下水用深度验证**(z+ → 深度增大则对,反则改 sign_z=-1)。

### ⑤ 代码驱动推进器 — 最低目标 ✅
通过 MANUAL_CONTROL(经 `pseudo_stick` / `diag_motor`)成功驱动全部 T200,arm 保持、退出自动上锁均正常。

---

## 3. 关键发现（对后续很重要）

1. **ESC 死区**:u=0.3 → 仅 ±24 µs,正好落在 T200 电调死区(~1500±25)→ 电机不转;u≥0.6(±48 µs)才转。
   → 之前"不转"完全是幅度问题,链路/映射一直正确。诊断/标定用 `--umax 0.6` 顶过死区。
2. **pilot gain 偏低**:即使 u=1.0 也只有约 ±80 µs(满程 ±400 的 ~10%)。
   → **下水做实际定深前需调高 ArduSub 输入 gain 或提高 U_MAX**,否则推力不足。
3. **MAVLink 心跳源**:必须锁定飞控 `sys1/comp1`(autopilot=3)。网络上还有 BlueOS 板载服务
   `sys1/comp191`、`sys1/comp194`(autopilot=8),其心跳的 mode/armed 字段是**无意义填充**,勿当真。
4. **失联失效时序**:解锁后若在 `input()` 阻塞(等操作),3s 内 FS_PILOT_INPUT 会 disarm。
   → 标定/交互流程需后台 keepalive 线程持续发指令+心跳。
5. **DVL**:Water Linked DVL 扩展的 `Enable DVL driver` 为关(未参与控制);干测应保持停用(避免过热,DVL 也靠水冷)。

---

## 4. 本次为真机适配的代码改动(均已推送 GitHub)

- `src/pseudo_stick.py`:`wait_heartbeat` 只锁定飞控心跳;新增 keepalive 线程(避免 input 阻塞期间失联)。
- `src/check_params.py`:只读 failsafe 参数查询。
- `src/diag_motor.py`:电机诊断(读 SERVO_OUTPUT_RAW / 心跳源 / STATUSTEXT)+ `--scan` 扫轴 + `--umax` 顶死区。
- `src/calibrate.py`:`--umax` 覆盖;keepalive 化。
- `config/vehicle.yaml`:manual_control 段标注干测验证结果。

---

## 5. 结论与下一步

**今日达成**:P0 连接 + P1 代码驱动推进器(最低目标)。轴映射确认为标准配置。

**留待下水那天**:
1. **z 绝对方向**:用深度响应验证 sign_z。
2. **调高 pilot gain / U_MAX**:让归一化指令产生足够推力。
3. **P3 系统辨识**:`sysid_collect` 采集阶跃 → `sysid_fit` 拟合 → 回填 `depth_model.yaml` 的 identified 段。
4. **P4 / P5**:用真实模型跑 PID 基线与 MPC 定深对比。

**收尾**:确认 DISARMED、断推进器电源。
