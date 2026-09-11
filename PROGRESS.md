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
- git commit: 2a7b040 (已推送)

### [Phase 2] 深度状态估计 (state.py + 噪声验证) — 2026-09-10
- 范围(经确认):只做 state.py + 噪声验证;控制器看门狗(超龄回中位)留到 P4。
- 决定:估计器用 **2 阶卡尔曼(恒速模型)**。
- 交付:
  - `src/state.py` —— `KalmanDepth`(变步长 KF) + `DepthEstimator`(时间戳/age/valid 封装);可 `python -m src.state` 对 SITL 打印估计。
  - `tests/sim_vehicle.py` +`--depth-noise σ`(高斯深度噪声,固定种子)。
  - `config/vehicle.yaml` + `state` 段(meas_sigma=0.03, process_accel_sigma=0.5, max_age_s=0.5)。
  - `tests/verify_state.py` —— 确定性数值验证。
- 验证结果(✅ 全部通过):注入 σ=0.030→反算 0.029;深度 RMSE 0.0295→0.0229(KF 降噪);速率 RMSE 0.135 m/s、不发散;速率时延≈100 ms(1 周期)。MAVLink 通路冒烟测试:含噪 SITL→state.py 输出平滑 z_kf/rate_kf,age≈0ms,valid=True。
- 权衡说明:process_accel_sigma 控制"平滑↔时延";当前偏响应(时延小)。真机标定 σ 后可再调。
- 仍未做(移交后续阶段):
  - **控制器侧看门狗**(深度超龄→回中位):age/valid 接口已就绪,但"回中位动作"在 P4 闭环里接。
  - **真机 σ 标定**:静止实测深度噪声,回填 meas_sigma(需真机)。
  - **真机深度到达率/时延实测**:确认 GLOBAL_POSITION_INT 实际 Hz 与抖动(需真机)。
- git commit: 5f6a557 (已推送)

### [真机准备] 解锁/上锁 + 方向标定 + 现场手册 — 2026-09-10(明天实测用)
- 背景:明天上机实测。决定:出水台架干测优先 / 代码解锁+自动上锁 / 目标到 P1(让推进器动起来)。
- 关键认知:真机上 **armed 才会驱动推进器**(仿真从不 arm),所以 P1 必须处理解锁。
- 交付/改动:
  - `src/pseudo_stick.py` + 解锁能力:`set_mode`(反向 mode 映射修复)、`arm(force)`/`disarm`、GCS 心跳、`--arm/--force-arm/--mode/--yes`;**退出/中断自动上锁**(fire-and-forget 多发上锁命令,不依赖收 ACK)。
  - `tests/sim_vehicle.py`:模拟真机 arm/mode(COMMAND_LONG/SET_MODE + ACK,heartbeat 反映 armed);**未 arm 时推进器不动**。
  - `src/calibrate.py`:引导式逐轴方向/中位标定,把 sign_* 写回 config(保留注释)。
  - `src/link.py`:自检小结加实测深度 Hz + 深度源 + 低速率告警。
  - `docs/FIELD_TEST.md`:现场手册(安全总则/BlueOS 端点/失联失效确认/P0→P1-a→P1-b→P1-c 步骤+通过标准+排查+收尾)。
- 离线验证(SITL,✅):未解锁指令无效;--arm 后 set MANUAL+解锁→指令驱动→退出自动上锁;标定写回 config;安全单元测试扩到 9/9(含解锁后自动上锁、未解锁不误上锁)。
- Windows 提醒:kill -INT 无法可靠投递 SIGINT,故 Ctrl+C→上锁改用确定性单元测试证明。
- 待明天真机确认:heartbeat/深度源与 Hz、z 中位与各轴符号、解锁是否需 force、ArduSub 失联失效行为。
- git commit: be60b3c (已推送)

### [真机准备2] 只读 failsafe 参数查询工具 — 2026-09-10
- 用户要求:明天到场先用只读方式查 failsafe 参数。
- 交付:`src/check_params.py`(严格只读,只发 PARAM_REQUEST_READ;默认读 FS_PILOT_INPUT/TIMEOUT、FS_GCS_ENABLE、FS_LEAK_* 等 failsafe 清单,带释义;--param 指定、--all 全量)。
- SITL 加 PARAM_REQUEST_READ/LIST 应答(SIM_PARAMS 示意值)供离线联调。
- 手册 FIELD_TEST.md 第 2 节改为"到场第一步:check_params 只读查 failsafe",并加入命令一览。
- 离线验证 ✅:对 SITL 读出 10 个 failsafe 参数,只读无副作用。
- git commit: 1466abf (已推送)

### [Phase 4] PID 定深基线(SITL)— 2026-09-10
- 目标:第一个深度闭环,PID 定深,建立 MPC 的对比基线;并接入 P2 推迟的控制器侧看门狗。
- 交付:
  - `src/pid.py`(PID:微分作用于测量 + 饱和条件积分抗饱和 + 限幅)。
  - `src/depth_control.py`(P4/P5 通用闭环:连接/解锁 + KF 状态 + 控制器可插拔 + **控制器侧看门狗(深度超龄→中位)** + 深度软限位 + CSV 记录)。
  - `src/metrics.py`(上升/超调/调节/稳态RMSE/控制能量/抗扰恢复)。
  - `tests/analyze_depth.py`(CSV→指标+出图)。
  - plant/SITL 加扰动力注入(`--disturb-force/-at/-dur`,plant.ext_force)。
  - 结果:`docs/RESULTS_P4.md` + `docs/baseline/`(图+CSV)。
- 基线指标(SITL,噪声0.02m):阶跃0.5→1.0m 上升1.02s、超调11.9%、稳态RMSE 0.037m、能量1.04;抗扰12N 峰值偏移0.127m、积分稳态u≈-0.14抵消、能量1.13。
- 踩坑:①启动瞬态——预热2.5s未控致浮力上浮冲出软限位→改短预热(KF初始化即控)+SITL 从水下 z0 起;②settle/recovery 用"最后离带"定义对噪声敏感→以 rmse_ss/峰值偏移为准。
- 控制器侧看门狗已接入并验证(STALE→中位;正常运行 0 次触发)。
- 待真机:PID 增益需按 P3 辨识/实测重调;真机噪声与到达率影响 KF/微分。
- git commit: e5ea12d (已推送)

### [Phase 5] MPC 定深 vs PID(SITL)— 2026-09-10
- 目标:CasADi 非线性 MPC 定深,复用 depth_control 框架,与 P4 基线同工况对比。
- 交付:`src/mpc.py`(NMPC:RK4 模型/输入硬约束/offset-free 扰动观测器/延迟补偿/IPOPT 热启动);config `mpc` 段补 dist_gain、delay_comp;`docs/RESULTS_P5.md` + `docs/baseline/compare_pid_mpc.png`。
- 最终对比(SITL 噪声0.02m,同噪声实现):阶跃超调 PID11.9% vs MPC10.6%;稳态RMSE 0.037 vs 0.040;控制能量 1.04 vs 1.40(PID更省);抗扰峰值偏移 0.13 vs 0.21m,稳态误差 1.6 vs 2.8cm(两者都offset-free);|u|≤0.3 MPC为优化内硬约束。
- 诚实结论:**1维SISO定深调好的PID很强,MPC难显著拉开(公认结论,非实现问题)**。MPC价值在硬约束/多DOF/参考预测,本工况未激发。保留基础设施,待真机P3辨识+约束场景重评估(与用户约定真机后调)。
- 调参历程记录:初版MPC含噪反差(超调14%、能量1.6)→ 定位为①扰动观测器×KF滞后致极限环 ②KF速度滞后+传输延迟致刹车晚;→ 加offset-free观测器(可配增益)+延迟补偿,权重Q_vel=3/R_du=1;最终与PID相当。
- git commit: 7302aeb (已推送)

### [Phase 3] 系统辨识框架(SITL 验证,待真机)— 2026-09-10
- 目标:先写好开环采集+拟合框架,真机入水采数据后回填 identified,给 MPC 用真实模型。
- 交付:`src/sysid_collect.py`(开环阶跃采集,复用解锁/安全)、`src/sysid_fit.py`(仿真误差最小化拟合 scipy least_squares + 回放验证图 + --write 回填)、`docs/RESULTS_P3.md`。
- 关键设计:只有比值可辨识→拟合 b_u/b0/a_lin/a_quad,固定 eff_mass 反算;**用仿真误差(只用深度、不微分)避免噪声放大**(初版对速度微分做线性回归 R²=0.44 很差 → 改仿真误差 R²=0.998)。
- SITL 验证(已知真值):深度回放 R²=0.998、RMSE 3cm;net_buoy 反算 -1.78(真值-2),b_u 2.61(真值3.08,数据有限偏低);流水线 采集→拟合→回放→--write→MPC自动切identified 端到端跑通。
- 修 bug:--write 正则误匹配注释里的 "identified:" → 污染 sim_truth;改行级定位真正段起始。
- 依赖:新增 scipy(requirements 已列)。identified 段仓库保持 null(真机回填)。
- 待真机:入水采集真实阶跃;可能需更丰富激励/标称 eff_mass/改进阻尼拆分。
- git commit: 92f75b3 (已推送)

---

## 真机实测 Day 1 — 2026-09-11(出水台架干测,目标 P0→P1)

### 环境 / 连接
- BlueROV2 R4 实机,**全程干测不下水**。
- 上位机 IP = **192.168.2.188**(以太网,BlueROV 网段);ping 192.168.2.2(飞控)通。
- 连接方式:BlueOS(Pirate Mode → MAVLink Endpoints)新建 **UDP Client → 192.168.2.188:14550**;上位机脚本用 `udpin:0.0.0.0:14550`。
- 坑:① 初次无 heartbeat = BlueOS 未配指向本机的端点;加 UDP Client 端点后解决。② Norton 防火墙需临时关闭/放行 python 收 UDP 14550。

### 步骤① check_params(只读 failsafe)— ✅ 完成
- 读数:`FS_PILOT_INPUT=2`(失联→disarm)、`FS_PILOT_TIMEOUT=3`s、`FS_GCS_ENABLE=2`(GCS 失联→disarm)、`FS_LEAK_ENABLE=1`、`FS_LEAK_ACTION`=未获取、`FS_CRASH_CHECK=0`、`FS_EKF_ACTION=0`、`BATT_LOW_VOLT=12`。
- 结论:**硬崩溃兜底充分** —— 进程崩溃/断网时 ArduSub 约 3s 内因 pilot input 或 GCS 心跳丢失自动 disarm;加上代码正常退出/Ctrl+C 主动上锁 = 双保险。可继续。

### 步骤② P0 link --check — ✅ 完成
- heartbeat OK,arm=DISARMED,模式=MANUAL,autopilot=3/type=12。
- 深度源=**GLOBAL_POSITION_INT**,实测 **11.0 Hz**;heartbeat 3.0 Hz。
- 深度静态噪声 ≈ **±5mm**(跨度 0.026m,干测无变化属正常),比仿真假设(20mm)更干净。

### 步骤③ 未解锁不动 — ✅ 完成
- 发 z=0.2(不 --arm)3s,T200 全程不转;安全默认(未解锁指令被忽略)验证通过。
### 步骤④ 轴映射/中位标定 — ✅ 完成(改用 SERVO_OUTPUT_RAW 判读,肉眼判不出)
- 工具:`src/diag_motor.py --scan`(依次扫 x/y/z/r,打印每轴 SERVO 增量)。
- 关键坑:u=0.3 只给 ±24µs,**正好在 T200 电调死区(~1500±25)→ 不转**;u=0.6→±48µs 顶过死区才转。用 `--umax 0.6` 顶过。
- SERVO 增量(通道1-8,u=0.6):
  - x=[+48,+48,+48,+48,0,0,0,0](水平同向=surge)
  - y=[-48,+48,+48,-48,0,0,0,0](sway)
  - z=[0,0,0,0,+48,-48,-48,+48](垂直差动=heave,Heavy 标准)
  - r=[-48,+48,-48,+48,0,0,0,0](yaw)
- 结论:**标准 BlueROV2 Heavy 分配矩阵,轴映射全部正确独立**。sign_x/y/z/r=+1,z_neutral=500(中位→1500 已验证)。config 保持默认。
- **绝对方向符号(尤其 z+ 是否=下潜)干测无法定,待下水用深度验证**(z+→深度增大则对,反则翻 sign_z)。
### 步骤⑤ 代码驱动推进器(最低目标)— ✅ 达成
- 通过 MANUAL_CONTROL(经 `pseudo_stick`/`diag_motor`)成功驱动全部 T200;arm 保持、退出自动上锁均正常。**"用代码给归一化指令让机器人动起来"完成。**
### 重要水下待办
- **pilot gain 偏低**:u=1.0 仅 ±80µs(满程±400 的~10%)。下水实际定深前需调高 ArduSub 输入 gain 或提高 U_MAX,否则推力不足。P3 辨识会把该缩放并入 K,但控制权限范围要够。
### 关键连接经验(真机)
- 必须锁定飞控心跳 **sys1/comp1(autopilot=3)**;网络上还有 BlueOS 板载服务 comp191/194(autopilot=8,心跳 mode/armed 字段是无意义填充,勿当真)。
- 解锁后若在 input() 阻塞,FS_PILOT_INPUT(3s)会 disarm → 需 keepalive 线程持续发指令+心跳。
### 今日未做(需下水):深度符号验证、P3 采集辨识、P4/P5 闭环、pilot gain 调整。
