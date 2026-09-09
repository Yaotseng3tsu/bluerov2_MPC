# 进度与监管记录 (Progress Log)

> 每个 Phase 的目的、命令/参数、实测结果、问题、决定都记录在此。每完成一步做一次 git 提交。

## 模板
```
### [Phase X] 名称 — YYYY-MM-DD
- 目标:
- 环境/前提:
- 执行的命令/参数:
- 实测结果:
- 遇到的问题:
- 你的决定 / 下一步:
- git commit:
```

---

### [Phase -] 项目初始化 — 2026-09-10
- 目标:搭建 `C:\bluerov2_mpc` 骨架、写 README/配置模板、本地 git 初始化。
- 已确认决定:① 首个闭环维度 = **深度 Heave**;② 仓库 = **新建 C:\bluerov2_mpc + 本地 git**;③ 控制通道 = **MANUAL_CONTROL**。
- 交付:README.md、requirements.txt、config/vehicle.yaml、docs/references.md、.gitignore、目录骨架。
- 下一步(待用户确认):进入 Phase 0,编写 `src/link.py` 连接自检并联机测试。
- git commit: 5fbe7fd (初始提交,已推送 github.com/Yaotseng3tsu/bluerov2_MPC)

### [Phase 0] 连接自检(离线部分)— 2026-09-10
- 目标:写好只读连接自检脚本,无真机时先用模拟器验证逻辑。
- 交付:`src/link.py`(只读自检,不发指令/不解锁)、`src/__init__.py`、`tests/sim_vehicle.py`(假 ArduSub)。
- 环境:C:\bluerov2_mpc\.venv;已装 pymavlink 2.4.49 / numpy / pyyaml。
- 执行:两进程离线联调 —— `python tests/sim_vehicle.py` + `python -m src.link --check --seconds 9`。
- 实测结果:✅ heartbeat 解析(autopilot=3, type=12 SUBMARINE, DISARMED, MANUAL);深度流 90 帧,跨度 0.999 m,符号约定(下潜为正)正确;小结判定逻辑正确。
- 已修:Windows cp932 控制台中文输出 → 脚本内强制 stdout/stderr utf-8。
- 待真机:等机器人到位,配好 BlueOS 14550 endpoint 后 `python -m src.link --check` 联机复测。
- 你的决定 / 下一步:**Phase 0 真机复测 + 进入 Phase 1(发指令)需机器人在场并经你同意**。
- git commit: b9b669b (已推送)

### [SITL] 搭建软件在环仿真(无真机开发)— 2026-09-10
- 目标:参考 HKPolyU 思路,先建深度动力学 + 闭环 MAVLink 仿真,让 P1–P5 离线可开发验证。
- 交付:
  - `src/plant.py` —— BlueROV2 深度(heave)动力学(有效质量/线性+二次阻尼/剩余浮力/推力,RK4);兼作 MPC 内部模型。
  - `config/depth_model.yaml` —— sim_truth 参数(eff_mass=26, c_lin=5.2, c_quad=37, K=80N, net_buoy=-2N);identified 段留给 P3。
  - `tests/sim_vehicle.py` —— 闭环 SITL:收 MANUAL_CONTROL → 积分 → 回传深度;raw socket(bind 14551→发 14550)规避 Windows udpout recvfrom(WSAEINVAL)问题。默认闭环,`--demo` 正弦供 P0。
  - `src/pseudo_stick.py` —— Phase 1 伪手柄:归一化 MANUAL_CONTROL + 限幅(U_MAX)+ 退出回中位 + 默认不 arm。
- 关键坑:pymavlink `udpout` 套接字不 bind,Windows 上 recvfrom 抛 WSAEINVAL → SITL 改用自绑定 raw socket + mav2 编解码。
- 实测结果(SITL 闭环 ✅):plant 终速与解析解一致(u=0.3→0.70 m/s);pseudo_stick 下潜 u=+0.3 深度上升、上浮 u=-0.3 回落、停后浮力漂移;限幅与退出回中位生效。**"让机器人动起来"最低目标在仿真中达成。**
- 你的决定 / 下一步:继续在 SITL 里做 P4 PID 定深 / P5 MPC 定深(离线);P3 系统辨识等真机数据。
- git commit: decb18f (已推送)

### [Phase 1] 安全逻辑严格验证 — 2026-09-10
- 目标:逐项验证 P1 安全逻辑(此前只跑了顺利路径,未严格验证)。
- 交付:`tests/test_safety.py`(7 项单元测试,mock 连接记录实际通道值)。
- 单元测试(7/7 ✅):
  1. 水平轴 U_MAX 限幅(u=0.9→300)
  2. z 限幅+映射(中位500, u=0.9→650, u=-0.9→350)
  3. 非标准 z 中位不越界(z_neutral=300 时 span 取小边,不越 [0,1000])
  4. 符号翻转 sign_z=-1 生效
  5. send() 端到端限幅(x=0.9,y=-0.9,r=0.9 → 300,-300,300)
  6. 退出回中位(close 发 5 帧中位,末帧=中位,连接关闭)
  7. 中断路径(main 内 KeyboardInterrupt → finally→close,rc=130,末帧=中位)
- SITL 集成测试(✅):
  - A 限幅端到端:请求 u=0.9,SITL 实收 u=+0.30。
  - B 退出回中位:cmd-timeout=20s 下,指令停后 u 立刻→0(来自中位帧,非超时)。
  - C 载具侧失效:硬杀 python(不发中位),约 1.5s(=cmd-timeout)后 SITL 自动 u→0。
- 无 arm 确认:pseudo_stick 只读显示 arm 状态,无任何解锁/改模式代码。
- 诚实说明 / 待真机确认:
  - "看门狗"在 P1 = (a) 退出/中断回中位 + (b) 载具侧 MANUAL_CONTROL 失联失效;**不是**控制器里的独立计时器。控制器侧带遥测超时的看门狗属于 P2+(需深度反馈)。
  - SITL 的 cmd-timeout=1.5s 是对 ArduSub 失联行为的**模拟**;真机上 ArduSub 的实际失联超时/是否 disarm 需 P1 现场确认。
  - z 中位=500、+z 下潜为 SITL 约定;真机符号/中位仍需 P1 实测。
- git commit: (P1 安全验证提交)
