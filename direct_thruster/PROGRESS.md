# direct_thruster 进度记录 (Progress Log)

> 本文件只记录 **direct_thruster 线**(修改并编译 ArduSub,实现外部独立控 8 个推进器)的进度流水。
> 目标 / 基线 / 构建环境 / 约束见 [`HANDOFF.md`](HANDOFF.md),完整方案与里程碑见 [`README.md`](README.md),实机基线见 [`M0_baseline.md`](M0_baseline.md)。
>
> waypoint 线(伪手柄定高前进)的进度在仓库根目录的 [`../PROGRESS.md`](../PROGRESS.md)。
> **两条线各写各的进度文件**(2026-10-06 从根 PROGRESS.md 拆出): 两边原先都往同一个文件尾部追加,
> 交替推进时必然在同一处打架; 拆开后文档边界与代码边界一致 —— 本线只动 `direct_thruster/`。

---

### direct_thruster M1 — 构建环境 (2026-09-17, dev 侧)
WSL2 Ubuntu-22.04 + ArduSub-4.1.2 clone(与实机 hash 一致) + 手动装构建依赖
(官方 `install-prereqs-ubuntu.sh` 在 22.04 会失败,去装 Python2 的包)。
卡点 = 验证 GCC10.2 交叉工具链 → 下一步 M2 编译 vanilla。**刷机须排在 waypoint 收尾之后。**

### direct_thruster M2 编译门槛 — ✅ 通过 (2026-09-17, dev 侧)
- `./waf configure --board navigator --toolchain $ARM_TC/bin/arm-none-linux-gnueabihf` + `./waf sub` 编译成功。
- 产物 `build/navigator/bin/ardusub` 1.9 MiB;`file` = **ELF 32-bit LSB ARM EABI5**,加载器 `/lib/ld-linux-armhf.so.3`(armhf,符合 Bullseye BlueOS)。
- 结论:**WSL + ArduSub-4.1.2 源码 + GCC10.2 工具链 + Navigator 构建链路全部可用**。
- **M2 尚未完成的另一半 = 装机验证**(把这个 vanilla 刷上 ROV 证明能启动)。受"刷机排在 waypoint 之后 + 用户显式同意 + 物理隔离推进器"门控。
- M3(改 C++)可在 dev 侧先做、不需刷机;但**不得在 vanilla 装机验证通过前刷改过的固件**。

### direct_thruster M2 装机验证 — ✅ 通过 (2026-09-17, 实机)
自编译 vanilla ArduSub 经 BlueOS `Upload custom firmware` 装入实机后, 三项被动验证(全程未解锁/未发控制信号)全部通过:
- **启动**: `fc_info` 拿到飞控 heartbeat sys=1/comp=1/autopilot=3/type=12; 版本 4.1.2 git 2dd0bb7d; **参数与 M0 基线逐项一致**(FRAME_CONFIG=2, SERVO1-8=Motor1-8/1100-1900/TRIM1500)。
- **遥测**: `yaw_monitor` ATTITUDE 240帧/12s = **20Hz**, 数值实时活动 → IMU/AHRS 正常。
- **输出**: `servo_monitor` 未解锁下 8 路**恒为 1500** 安全中位。
- **结论: "电脑编译成功 ≠ ROV 能启动" 这关过了** —— GCC10.2/armhf 选择正确, glibc 兼容无问题。M3 之后若出问题可确定是自己的 C++ 而非构建环境。
- 新增只读工具: `direct_thruster/servo_monitor.py`(被动看8路输出)、`direct_thruster/fc_link.py`(锁定真飞控心跳)。
  **修了一个工具 bug**: 原 fc_info 用朴素 wait_heartbeat() 会锁到 BlueOS 服务(sys=0)导致参数全读不出 → 改为只认 autopilot!=INVALID 且 type!=GCS。
- 未做: 解锁后 8 路响应测试(需拆桨); BlueOS 固件页是否标注自定义(佐证跑的是自编译份)。

#### M2 订正: 装机验证须在 RESTART AUTOPILOT 之后才算数
- **用户发现的关键漏洞**: BlueOS "Successfully installed new firmware" 只说明**文件写入磁盘**, 正在跑的 ArduSub **进程仍是旧的**, 直到点 `RESTART AUTOPILOT`。
- 更麻烦的是**版本号无法分辨新旧**: 我们编的是同一个 tag, 自编译版同样报 `4.1.2 (STABLE)` / git `2dd0bb7d`, 连 BlueOS 页面显示都一样。
- **客观判据 = autopilot uptime 归零**(`fc_info` 已加打印)。重启后 uptime≈0 → 确认进程已换成磁盘上那份。
- 结论: 重启后复测 heartbeat/版本/参数全部正常 → **M2 装机验证坐实**。重启前那次验证不作数。
- 教训记入流程: 以后每次刷机, **装完必 RESTART + 看 uptime**; M3 起在固件里加开机 STATUSTEXT 标记, 一眼确认跑的是哪份。

### direct_thruster M3 — C++ 改动完成并编译通过 (2026-09-17, dev 侧, 未刷机)
在 WSL 的 `ardupilot-external`(分支 `external-thrusters`) 实现"外部 8 路直控", 7 文件 +214 行, 编译通过(1.9MiB ARM EABI5)。diff 存 `direct_thruster/patches/0001-external-8ch-thrust.patch`。

**设计(读真实 4.1.2 源码后, 对原计划有三处修正)**:
- **切入点在派发器顶部** `AP_Motors6DOF::output_armed_stabilizing()` 最上面拦截 → **三个混控函数一行未动**(你的 Heavy 实际走 `output_armed_stabilizing_vectored_6dof()`)。
- **只写 `_thrust_rpyt_out[]`, 绝不直接 rc_write** → 下游 `output_to_motors()` 的 spool 门控仍在, **未解锁时输出被硬写 1500**, "未解锁发非零命令不动桨"是结构性保证而非靠 if。
- **必须自己补 `_motor_reverse[i]` + 限幅**(混控最后一行做的事), 否则 `MOT_n_DIRECTION` 会静默失效。
- **`get_current_limit_max_throttle()` 在 4.1.2 里恒返回 1.0(空壳)** → 原计划担心的"丢了总电流限制"在此版本无实质内容, 不需处理。

**接口**: `SET_ACTUATOR_CONTROL_TARGET` + 私有约定 `group_mlx=1` = Motor1..8, `controls[i]∈[-1,1]`。
校验: sysid==SYSID_MYGCS / target 匹配 / group==1 / MOT_EXT_ENABLE 开 / 8 路全部 finite 且在 [-1,1] → **整帧拒收, 不部分采纳**。
**新参数**: `MOT_EXT_ENABLE`(0/1, 默认0) `MOT_EXT_TMOUT`(ms, 默认500)。

**failsafe**:
- 合法外部命令会刷新 `last_pilot_input_ms` → **保留原 pilot-input failsafe 而不是关掉它**(否则外部模式下不发 MANUAL_CONTROL 会被判"Lost manual control"而 disarm)。
- 新增 `failsafe_ext_thrust_check()`(50Hz): 超时 → 归中位 + **disarm** + **锁存故障**; 不恢复原 mixer、不沿用上一条命令; 清 `MOT_EXT_ENABLE` 才解锁存。**故意不做 SITL 编译屏蔽**(原 pilot 检查有 `#if != SITL`, 在 SITL 验不出来)。
- 开机 STATUSTEXT `"EXT-THRUSTER build..."` → 解决"版本号/hash 与官方完全一致、分不清跑的是哪份"的问题。

**未做**: 尚未刷机(等时机); 验收四项(未解锁发非零/单路独立/看门狗/非法值) 待 SITL → 断桨干测 → 水下。
