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

### direct_thruster M4 — SITL 验收台重建 (2026-10-06, dev 侧, 未刷机)
首轮 SITL 验收 0/11, 诊断结论是**测试脚本的问题, 不是固件** —— 证据是 D2 项里
`SERVO3=[1500, 1740]`, 外部命令确实驱动了 Motor3。本次只重建验收台, **固件 C++ 一行未动**
(fork commit 仍是 `d88e653`)。

**四个根因, 逐条对应修法**:

1. **`reset_ext()` 有竞态, 照原样重跑必然还是红**。固件里"解锁存"和"清残留命令"是两条独立路径:
   `clear_external_fault()` 只在 `MOT_EXT_ENABLE==0` 时被 50Hz 检查调用,
   `clear_external_thrust()` (清 `_ext_have_cmd`) 只在看门狗真正触发时调用。
   直接 `ENABLE 0->1` 的话, `_ext_have_cmd` 仍是 true 且 `_ext_last_ms` 早已过期,
   重新使能那一瞬 `external_thrust_timed_out()` 立即为真 -> 20ms 内当场重新锁存。
   修法: 先让看门狗在 ENABLE 还开着时打一次(它会清掉 `_ext_have_cmd`), 再 `ENABLE 0 -> 1`。
2. **SITL 默认是 6 推进器 Vectored 帧**, Motor7/8 不被使能、SERVO7/8 恒 0,
   把所有"全中位"判据整体带崩(这就是 0/11 的直接原因)。修法: `FRAME_CONFIG=2` 起 SITL。
3. **被拒收的测试项会落回原混控**, 而 SITL 里没人喂 RC, `norm_input()` 钳到 -1 会把桨推离中位,
   让"全 1500"因与本功能无关的原因报红。修法: 全程 10Hz 发中位 `MANUAL_CONTROL(z=500)` 把混控钉住,
   并加一条基线前置检查(关掉外部控制 + 解锁 + 中位手柄 -> 必须全 1500), 不过就 abort。
4. **窗口串扰与启动瞬态**。换命令瞬间, 上一窗口残留在接收队列的报文会算到新窗口头上;
   启动瞬态读数 1000 会污染 A 项。修法: `run()` 加 `settle` 段(照常发命令但丢弃采样) + 开场 3s settle。

**新增/重写的文件**:

| 文件 | 作用 |
|---|---|
| `sitl_ext.parm` | SITL 追加默认参数: `FRAME_CONFIG=2` / `MOT_EXT_*` / `SR0_RC_CHAN=20`。全 ASCII 且每行 <80 字节 —— `AP_Param` 的 defaults 解析器用 `char line[100]` 固定缓冲, 中文注释会被 `fgets` 截断, 后半截当成参数名解析导致整个 defaults 加载失败 |
| `sitl_run.sh` | 一键: 杀残留 -> 擦 eeprom -> `-w` 起 SITL(带两个 defaults) -> 等端口 -> 跑验收 -> 收尾。`-w` 必须排在 `--defaults` 前面(SITL 按 argv 顺序处理选项, `-w` 当场 `erase_all()`)。等端口用 grep 日志而非 TCP 探测, 因为 5760 一次只收一个客户端, 探测连接会和验收脚本抢名额 |
| `sitl_accept.py` | 重写测试主体。前置条件改为**硬 abort**(改版参数在位 / `FRAME_CONFIG==2` / 电机通道=={1..8} / 基线全中位) —— 首轮的教训是前提错了还照常跑完, 会产出一堆看不懂的 FAIL |
| `.gitattributes` | `*.sh` `*.parm` 强制 LF。本仓库 `core.autocrlf=true`, CRLF 会让 WSL 的 bash 报 `bad interpreter`。作用域只限 `direct_thruster/` |

**计分项从 11 条改为 14 条**, 其中新增的是直接对应"八路单独控制"这个目标的:
`B3` 负向命令(验 `_motor_reverse` 与符号)、`B4` 八路逐一扫描(每路单独给 0.5, 验其余七路恒中位)、
`B2` 推力标度(`SERVO3 ~= 1740 = 1500 + 0.6*400`)、`D4` 错误 `target_system` 拒收。

**状态: 验收台重建完成, 但尚未运行。** 四项验收的实际结果待下一步
`bash direct_thruster/sitl_run.sh` 产出; 在那之前 M4 不能算通过。
刷机仍受"用户显式同意 + 排在 waypoint 收尾之后"门控(拆桨隔离这一条用户已于 2026-10-06 放宽)。
