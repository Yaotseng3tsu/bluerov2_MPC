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

---

## Phase W — 伪手柄 A→B 点到点前进(DVL 航位推算) [2026-09-16 立项]
新增子模块 `waypoint/`,目标:无外部绝对定位下,伪手柄让 BlueROV2 前进指定距离(例 3m)后停。
方案 = **surge 动力学模型算前馈指令** + **DVL 速度航位推算(∫vx·dt)判到距停** + **IMU/ATTITUDE 航向保持走直线**。复用主项目 `pseudo_stick`/`plant`/`sysid` 范式与安全约定。

### 数据结论(为何要补辨识)
- 深度模型只标定 heave,水平轴分配/附加质量/阻尼不同,不能挪用。
- `rl_logger` 拖曳 IMU 24 组是被动拖曳(无遥杆指令通道),只含姿态/起停/方向可分性,无"指令→运动"映射。
- ⇒ 必须先补一小段 **surge 系统辨识**(W2);有 DVL 速度反馈后成本很低。

### 决策(2026-09-16)
- 定位策略:**DVL 航位推算**(用户选;比纯开环鲁棒,仍无绝对位置真值)。
- 新文件夹:`C:\bluerov2_mpc\waypoint\`(子模块,复用主项目脚手架)。
- W2 辨识档位**顶过 ESC 死区**:`u_x=0.35/0.45/0.55`(前后各点动),采集须 `--umax 0.6` 覆盖 U_MAX=0.3。
- W1 **保留 BlueOS DVL 扩展**(不停用):优先直连 16171 只读,占用则退回 MAVLink 读扩展转发速度。

### 步骤规划(每步先经用户确认再推进)
- W1 DVL 接入自检(dvl_stream.py) / W2 surge 系统辨识(surge_sysid_collect+fit → surge_model.yaml)
- W3 surge_plant + 前馈梯形速度轨迹 / W4 heading_hold(ATTITUDE.yaw,P) / W5 go_distance.py 主脚本 / W6 干测→水下→复盘

### 当前状态
- ✅ 已建 `waypoint/` 与 `waypoint/README.md`(含架构图/文件规划/分步计划/安全)。下一步待确认后进 W1。

### Phase W 范围升级 (2026-09-16): 前进3m → 相对航点定向航行 (Point-and-go 4DOF)
- 澄清: BlueROV2(Heavy) 只独立控 4DOF (surge/sway/heave/yaw); roll/pitch 被动稳定, 只监测不控。
- 范式 (用户选): **Point-and-go 4DOF** —— ①调深度 ②转航向 ③沿航向前进到距离; sway 接口预留 (holonomic 扩展)。
- 闭环来源: surge=DVL vx 航位推算; heave=绝对深度(复用现有模型, 真闭环最准); yaw=ATTITUDE; 世界系积分(R(yaw))。
- README 重写为该范围。

### W3 surge 运动模型 + 前馈梯形轨迹 — ✅ 完成 (全离线)
- 新增 `waypoint/config/surge_model.yaml` (sim_truth 占位 eff_mass17/c_lin4/c_quad18/K50; identified+sway 留 null)。
- 新增 `waypoint/motion_model.py`: SurgePlant(RK4) + TrapezoidTraj(DOF无关梯形速度) + 逆动力学前馈 feedforward_cmd。
- 离线自检 (dist3/v0.4/a0.15): 剖面积分3.000m, 开环终点3.002m(+0.1%), 末速+0.001m/s。出图 data/motion_model_selftest.png。
- **发现**: sim_truth 占位 K=50 下前馈 |u| 峰值仅 0.137, 落在 ESC 死区(~0.3)内。真机 K_x 更低→u 更大, 待 W2 定; 无论如何 W5 需加**死区补偿**或选 u>0.3 的巡航/加速。

### W4 航向保持 heading_hold — ✅ 完成 (全离线)
- 新增 `waypoint/heading_hold.py`: HeadingHold(PD, 误差 wrap 到 ±180 走最短转向, r 限幅) + 仅自检用 YawPlant。
- 默认: kp1.0/kd0.2/r_limit0.3/tol±3°。自检: 0°→90° 3.6s 到位无超调; 170°→-170° 识别为+20°最短转向 1.7s 到位。PASS。
- 出图 data/heading_hold_selftest.png。注: r_limit0.3 处于 ESC 死区边缘, 真机偏航可能偏弱(W6 调)。

### 仿真件 (sim harness) — ✅ 完成 (离线, 为 W5 集成测试铺路)
- `waypoint/sim/fake_dvl.py`: 假 A50 TCP 服务(16171), 吐与真机同格式 velocity JSON; 速度由回调提供; 独立可发恒定/正弦。
- `waypoint/sim/sim_waypoint.py`: 4DOF SITL(MAVLink+假DVL一体), 解 MANUAL_CONTROL 的 ux/uz/ur → SurgePlant/DepthPlant/YawPlant 积分; 回传 HEARTBEAT/GLOBAL_POSITION_INT/ATTITUDE; DVL vx=surge机体速度。含 --current-vx/--dvl-bias/--no-dvl 供测鲁棒性与降级。
- 联测: fake_dvl↔dvl_stream 10Hz PASS; sim_waypoint 遥测 HB/POS/ATT 正常, yaw0=30°→ATTITUDE读30.0°。
- 未改动 tests/sim_vehicle.py (深度 SITL 9/9 保持)。

### W5 主脚本 go_waypoint — ✅ 完成 (逻辑离线跑通; 末端精度调参待连机)
- `waypoint/go_waypoint.py`: 4DOF 状态机 DESCEND→TURN→CRUISE→STOP→DONE。
  深度=PID绝对深度闭环(复用src/pid+state); 航向=heading_hold(ATTITUDE); 前进=motion_model前馈+DVL航位推算(∫vx·dt)到距停。
  复用 pseudo_stick 安全; 含死区补偿 deadzone_comp、--umax 覆盖、--vx-sign、--rel。
- SITL 端到端验证 (sim_waypoint+假DVL):
  - 基线 h90/d3/z0.5: DONE REACHED, s=3.00m, depth 稳0.50, yaw 稳90。出图 data/go_waypoint_base.png。
  - 水流6N: DONE REACHED s≈3.0 (DVL 航位推算对水流鲁棒), 残余v≈0.48(无位置控, 诚实报告), depth 保持0.40。
  - DVL偏大20%: 停在 DVL 测得3m(真实~2.5m)——演示 DVL 标定误差直接进位置误差(W1须定标/定符号)。
  - DVL失效(--no-dvl): CRUISE 入口即 DVL_LOST→拒绝盲走, s=0, 深度/航向仍保持。
- 修复(sim 中发现): ①STOP 分紧急(全中位)/正常到达(保持深度航向,只切前进); ②稳停超时兜底(水流下vx不归零); ③STOP 阶段持续重算深度/航向(原冻结致漂移)。

### 离线路线小结
W1代码/W3/W4/仿真件/W5 全部离线跑通。剩余均需连机: W1真验证(vx符号/更新率/直连16171)、W2真实采集辨识、W6干测→水下→复盘。

### W2 surge 系统辨识 (采集+拟合) — ✅ 代码完成 (真实参数待连机采集)
- `waypoint/surge_sysid_collect.py`: 开环阶跃(默认±0.35/0.45/0.55 顶过死区) + DVL 记 vx; 含**距离护栏**(段内 |s|>--max-dist 回中位防撞墙)、DVL 丢失跳段、--umax 覆盖、--vx-sign。深度靠操作者维持(只发 x)。
- `waypoint/surge_sysid_fit.py`: 仿真误差最小化拟合 vx(t) → (b_u,b0,a_lin,a_quad); 固定 eff_mass 反算 K/c_lin/c_quad; b0=残余加速度仅报告; --write 回填 surge_model.yaml identified。
- **SITL 验证管线**: 对 sim_truth(K50/c_lin4/c_quad18/m17) 采集拟合, R²=1.000, 恢复 c_lin4.01/c_quad16.1/K46.1(≤10%), 终速1.08 vs 1.13。--write 写入/还原均正常。
- 遵守约定: SITL 值未入库 (identified 保持 null, 待真机)。
- 顺带修 sim: DVL valid 恒 True(底锁与解锁无关, 更贴近真机 A50)。

### 离线路线 100% 完成
W1代码 / W2代码 / W3 / W4 / 仿真件 / W5 全部离线跑通并验证。连机那天为纯执行:
W1真验证(vx符号/直连16171/更新率) → W2采集(surge_sysid_collect --arm)→拟合(surge_sysid_fit --write)→ W5(go_waypoint 直接用 identified 参数) → W6 干测/水下/复盘。

### W6 连机 Day (2026-09-16) — 下水前只读检查 ✅
- 链路: heartbeat sys1/comp1 autopilot=3 DISARMED/MANUAL; 深度源 GLOBAL_POSITION_INT @26Hz 噪声±5mm。
- DVL: 直连 16171 成功(无需停用 BlueOS 扩展); 气中 valid=false/alt=-1(正常), 更新率≈4.7Hz(气中偏低, 待下水复测)。
- failsafe: FS_PILOT_INPUT=2/TIMEOUT=3/GCS=2/LEAK=1 硬兜底充分。
- 待下水项: vx 前进符号(需底锁)、W2 采集辨识、W5 航行。记录进 waypoint/RESULTS.md。

### W6 干测 (2026-09-16) 续 — 干测能验的全部完成 ✅
- 航向传感器: yaw_monitor 手转验证, ATTITUDE.yaw 全圈跟随[-180,+176]跨度356° @20Hz, roll/pitch 实时 → heading_hold 输入可靠。
- go_waypoint 全链路(不解锁)真机联调: 预热/DESCEND→TURN/ur=+0.30/CSV/超时退出 均正常。
- 解锁短点动扫描(diag_motor --scan --u0.6 --umax0.6): SERVO 增量全符合标准 Heavy(x[48×4]/y/z/r), 与 Day1 一致 → 指令通路能驱动正确电机。
- ⚠️ 推力权限偏低: u=0.6→±48µs(满程~12%), 明天下水前建议调高 ArduSub pilot gain 或用 --umax 0.8~1.0。
- 待明天下水: vx符号、DVL下水更新率、深度闭环、heading_hold极性(+r→yaw增?)、surge绝对前进、W2采集辨识、W5航行。

### W6 下水 Day (2026-09-17) — 集成 DVL 观测工具 + 下水 DVL 确认
- 从 rl_logger 移植(非重写, CSV 字段一致)3 个观测工具到 waypoint/: dvl_dashboard.py(实时网页仪表盘+双CSV)、dvl_traj_log.py(无界面记录)、plot_dvl_traj.py(离线出图)。仅改 outdir→waypoint/data/dvl_data。
- 实机水中验证: 移植后 dashboard 直连真 DVL OK; 底锁 alt=1.49m/fom=0.0014/valid=True; velocity+position_local 双路都通。
- **A50 多客户端共存已验证**: dashboard 与控制环 dvl_stream 同时连 16171 都收到数据 → go_waypoint/sysid 运行时可同时开 dashboard 观测。
- 控制环仍用 dvl_stream(velocity-only 线程安全, 不改); dashboard 是并行观测层。
- W1 下水复测: DVL 更新率≈4.4Hz(气中水中一致), 底锁确认。

### W6 下水 (2026-09-17) — sign_z 定案 + 高度控制
- **sign_z 水中实测为反**: z_move 上浮/下潜颠倒 → `config/vehicle.yaml` 改 `sign_z: -1`(Day1 挂起的问题今天定掉)。
- **增益根因**: JS_GAIN_DEFAULT=0.2 / JS_GAIN_MAX=0.5(顶格才 50%)→ 改为 0.8 / 1.0;实测 u≈0.8 可快速上浮。
- 新增 `waypoint/z_move.py`: z 轴上浮/下潜(MANUAL 直接推力)+ 实时深度 + 自动判读 sign_z;退出交回 ALT_HOLD 不 disarm。
- 新增 `waypoint/altitude_hold.py`: **高度闭环**(PID + DVL altitude + 伪手柄)。反馈用离底高度而非压力深度;把 -alt 当深度复用已验证 PID;`--u-bias` 前馈补负浮力;DVL 丢底锁/超龄/越界即停;退出交回 ALT_HOLD 保持 armed。
- 仿真同步: sim_waypoint 的 z 解码改为匹配真机极性, 并让假 DVL 高度 = bottom - depth、新增 --net-buoy 模拟负浮力。
- SITL 验证(负浮力 5N): 高度 1.0→目标 1.2 收敛至 1.202m(误差 -0.002m), 稳态 u_z≈-0.065。

### W6 下水 — 定高前进 go_forward (高度PID + 距离PID 双闭环)
- 新增 `waypoint/go_forward.py`: 阶段 HOLD(稳高度)→ADVANCE(高度+距离同时控)→DONE(反推刹车并稳住位置)。
  - 高度 z: 复用 altitude_hold 那套(-alt 当深度 / u_bias 前馈 / slew / 抗饱和)。
  - 距离 x: s=∫vx·dt, u_x=Kp(sp-s)+Ki∫e-Kd·vx —— **D 项直接用 DVL 实测 vx,免数值微分**;
    距离设定值按 --v-cruise 匀速斜坡 → 匀速前进不冲过头。
- SITL: 高度末态误差 +1mm; 距离最初超调 0.24m(DONE 阶段把 u_x 清零→靠惯性滑行),
  改为 **DONE 仍跑距离 PID(反推刹车+位置保持)** 后超调降到 0.075m。
- **安全: vx 符号自检**(--vx-sign 若反, s 会往负走、max_dist 护栏永不触发 → 会一路撞墙)。
  加了 s<-0.3 与"推前进却持续后退"两道拦截; 故障注入(--vx-sign -1)验证在 s=-0.38 正确中止。
- 待真机: vx_sign 仍未实测(此前增益太低没动), 建议 dist 0.5→1.2→2.0 渐进。

### W6 下水 — go_forward 三闭环 + 设定值节流 (实测问题修复)
- 真机 f05 实测暴露两问题: ①到不了目标距离 ②上升/前进时 yaw 偏移大。
- 诊断(读 CSV): **vx_sign 是对的(+1)**; 但 `--v-cruise 0.15` 远超机器人实际能力(~0.06m/s,
  u_x 全程顶死 0.6) → 设定值跑飞, 滞后达 **1.178m**, u_x **74%** 时间饱和, 距离 PID 退化成开关控制;
  按实际速度走完 2m 需 ~33s + HOLD 18s, 而该次 39.7s 就结束 → "时间不够"只是表象。
- 修复: **设定值节流** —— 只有"设定值−实测"滞后 < --max-lag / --alt-lag 时才推进设定值,
  自动适配机器人真实速度。SITL: 滞后 0.094m, u_x 饱和 0%。
- 新增**第三路航向 PID**: 读飞控 ATTITUDE.yaw(不用会漂的 DVL yaw), HeadingHold(PD)→r,
  锁定解锁时航向; --no-yaw 可关。
- 默认值按实测调整: v_cruise 0.15→0.08, x_limit 0.6→0.8, seconds 120→180。
- SITL(负浮力+初始航向20°): 高度末态误差 0.000m, 距离 2.069m, 航向全程零误差。

### W6 下水 — f2m 实测分析与优化 (推力门槛 + 垂直/偏航耦合)
- f2m 结果: 走到 1.975/2.00m(误差2.5cm), 高度巡航段 0.80±0.03 —— **任务基本达成**, 用时 71.8s。
- **问题1 推力门槛**: 实测 u_x→vx 为 0.3~0.4→0.013(几乎不动) / 0.5→0.031 / 0.6→0.060 / 0.7→0.085。
  巡航 0.08m/s 需 u_x≈0.68; 但节流把滞后卡在 0.25m, Kp=1.0 只给 0.25 指令(低于门槛),
  全靠 Ki=0.05 慢爬 45s 才堆到 0.67 → 中段爬行。
  **修**: 加 surge 速度前馈 `--x-ff`(默认0.60, 直接跨过门槛; 设定值到目标后平滑衰减到0以免妨碍刹车);
  Kp 1.0→1.5。
- **问题2 垂直推力→偏航耦合**: 负浮力使 u_z 常年 ~-0.6, 垂直桨产生**恒定反扭矩**;
  HeadingHold 原为纯 PD, 对恒定扰动必留稳态误差, 且 r_limit=0.5 在上升段**饱和65%**仍被转走 **80°**。
  **修**: HeadingHold 加积分(ki, 含条件积分抗饱和); go_forward 默认 --yaw-ki 0.20, --r-limit 0.5→0.9。
- **仿真局限(记录)**: sim 的 surge 在 u=0.6 终速 ~1.2m/s, 真机仅 0.06m/s(差 ~15x);
  垂直亦需 --net-buoy 48 才匹配真机悬停推力。**水平参数只能按真机实测定, SITL 仅验机制。**

### W6 下水 — 发现并修复深度源污染 BUG (用户观察触发)
- 用户观察: DVL altitude 正确, 但控制用的 depth 数值不对。
- **实测确认**: parse_depth 同时接受 GLOBAL_POSITION_INT / VFR_HUD / SCALED_PRESSURE2,
  调用方"收到哪条算哪条", 并未按 config 的 depth_message 过滤。现场实测:
    GLOBAL_POSITION_INT = +0.866m (真值, 10Hz)
    VFR_HUD.alt         =  0.000m (字段未填, 10Hz)
  → 喂给卡尔曼的深度在 0 与真值间每 100ms 跳一次, 深度测量彻底失效。
- **影响**: 早上 MANUAL 深度 PID 控不住, 我当时归因于增益/sign_z —— **测量本身也是坏的**;
  "解锁时深度基准跳变 0.844→0.276" 的结论**是错的**, 实为 0↔0.87 交替被滤波平均的假象(已撤回)。
  altitude 走 DVL 不经此路, 所以一直正确 —— 与用户观察一致。
- **修复**: parse_depth 增加 allow 参数(按 config depth_message 过滤), 全部 6 处调用点传入。
- 顺带修: tests/sim_vehicle.py 的 z 极性未随 sign_z=-1 同步(之前只改了 sim_waypoint) → 已对齐真机。
- 回归: 深度 SITL 正常定深 0.60m; 安全测试 9/9 通过。

### W6 下水 — "DVL 丢底锁/超龄" 误中止的根因与修复
- 现象: ff2m 第二次运行 2.9s 即以 "DVL 丢底锁/超龄" 结束。
- **表层**: 机器人已沉到 alt≈0.09-0.19m 并剧烈跳动, 接近 A50 最小工作高度(~0.05-0.1m);
  且 u_z 满推 -1.0 在底部搅起气泡/泥沙, 正打在换能器下方 → 间歇解算失败。
- **深层(代码缺陷)**: dashboard 原始记录 11562 帧中 **286 帧(2.5%) valid=false/alt=-1**。
  而 `is_fresh()` 只看**最新那一帧**, 10Hz 控制环几秒内必然撞上一帧坏数据 → 误判丢底锁中止。
- **修复**: DvlStream 单独保留 `_latest_valid`(最近一帧有效样本), 新增 `latest_valid()`;
  `is_fresh()` 判据改为"最近一次**有效**帧的龄期 <= max_age", 容忍瞬时丢帧;
  altitude_hold / go_forward 的控制环改用 `latest_valid()`(丢帧时沿用上一帧好数据)。
- 故障注入验证: sim 新增 `--dvl-dropout`; 注入 5%(2倍于实测)无解帧, 任务完整跑完
  (高度误差 0.000m, 距离 1.904/2.00m) —— 修复前会在数秒内中止。

### W6 下水 — 【根因】Cockpit 手柄与脚本抢 MANUAL_CONTROL
- 现象: `z_move --dir up --u 0.8` 推不动, 但用户轻推 Cockpit 手柄机器人就上升。
- **实测证据**(只读监听链路 12s):
    MANUAL_CONTROL 来自 sys255/comp240: 300 条 ≈ **25 Hz**, z=475~498(中位微抖) ← Cockpit 手柄
  我们的脚本 10Hz。飞控只认**最后到达**的一条 → 约 70% 周期被手柄的中位覆盖,
  指令被稀释, 表现为"满推也推不动"。
- **推翻先前判断**: "u_z=-1.0 满推仍下沉" **不是硬件故障**(进水/缆拖拽/桨缠绕),
  而是指令冲突。早先 altitude_hold 能稳在 0.8m, 是当时手柄未发; 之后手柄活跃即全面失效。
  用户从一开始反馈的"感觉在打架"是对的。
- **修复(代码侧)**: PseudoStick 新增 `detect_rival_manual_control()` / `warn_if_rival()`,
  z_move / altitude_hold / go_forward 解锁前自动检测并告警(可选择中止)。
- **操作规程**: 跑本项目脚本前必须在 Cockpit/QGC **断开或停用手柄**。

### W6 下水 — 手柄冲突解除后复测: z_move / go_forward 均正常
- 断开 Cockpit 手柄后重跑 `z_move --dir up --u 0.8 --seconds 6`:
  深度 0.883 → 0.405 m(**6s 上浮 0.478m, ≈0.085 m/s**), z_ch=900 全程稳定。
  → 之前"z_move 不上升而 go_forward 能上升"同样是**手柄抢信号**, 两脚本无差异(走同一 send 路径)。
- **sign_z = -1 得到二次确认**(命令上浮 → 深度变小)。
- 修 z_move 判读文案 BUG: 原文硬编码"保持 +1", 但 config 已是 -1, 照做会把方向改反;
  改为读取当前 `manual_control.sign_z` 并提示"保持当前值 / 需从 X 改为 -X"。

### W6 下水 — 传感器跳变导致的两个危险行为及修复
- **① 航向跳变 → 机器人急速旋转(危险)**: 实测 t=15.8→16.2s 内 yaw 由 -0.7° 跳到 +57.7°
  (58°/0.4s = 145°/s), 而当时 u_r 仅 0.25(几乎没在转) → 是**航向估计跳变**(EKF重对准/罗盘),
  但控制器当真, 立刻打到 u_r=-0.84 满舵 → **把机器人真的抡起来**。
  **修**: drain_attitude 用同报文的陀螺 `yawspeed` 交叉校验, 角度跳变超过
  `(yawspeed + --yaw-jump-dps)*dt` 即剔除(沿用上一个好航向);
  连续坏 > `--yaw-stale`(1.5s) 则**停用航向控制**(u_r=0), 不拿坏估计驱动推进器。
- **② DVL 高度外点 → 直接中止**: altitude 偶发跳到 2.5m/3.3m, 撞 alt_max 护栏结束任务。
  **修**: DvlStream 增加高度跳变剔除(--max_alt_rate 1.5 m/s), 连续 5 帧仍偏离才认账(真换地形)。
- 故障注入验证(高度外点3% + 航向跳变5% + 丢帧2.5%): 任务完整跑完,
  |yaw_err| max 0.0°、|u_r| max 0.00(无虚假打舵), 日志中无 2.5m 外点。

### W6 下水 — 航向跳变剔除(第二版): 参考系平移不该打舵去追
- 第一版失效: `allow = (yawspeed + jump_dps) * dt`, 而 `yaw_t` 只在采纳时更新 →
  跳变持续时 dt 一直变大、allow 跟着涨, **约 0.9s 后 108° 的门限就放过了 105° 的跳变**,
  控制器随即算出 -104° 误差、打到 u_r=-0.90 满舵(实测 t=17.5s)。
- 第二版(根本修法):
  ① `dt` 钳位 0.2s, 门限不再随拒收时间膨胀;
  ② 连续 `--yaw-accept-n`(5) 帧仍偏离 → 认定为**航向参考系整体平移**(EKF 重对准),
     **把 hh.target 同量平移**并清零航向积分 —— 物理指向未变, 就不该转机器人;
  ③ 仍保留 `--yaw-stale` 超时停用航向控制作为兜底。
- 故障注入(t=20s 起永久 +105° 参考平移 + 尖刺5% + 高度外点3% + 丢帧2.5%):
  平移瞬间 |u_r|max=0.00、|yaw_err|max=0.0°(**无打舵**), 任务完成(高度-6mm, 距离0.980/1.00)。

---

## 现状快照 (2026-09-17) — W6 下水实测后
**主用脚本演进**(实机定高作业更实用): `go_forward.py`(高度PID + 距离PID + 航向PID 三闭环, 定高前进; 设定值节流) · `z_move.py`(垂直直推, 定 sign_z) · `altitude_hold.py`(DVL 离底高度定高)。新增 DVL 观测工具 `dvl_dashboard.py`/`dvl_traj_log.py`/`plot_dvl_traj.py` · `hold_jog.py`。`go_waypoint.py` 仍在(点到点 4DOF)。

**关键实机结论**:
- **sign_z = -1**(命令上浮→深度减小, z_move 二次确认); 负浮力机器人需 `--u-bias` 恒定上推力才能悬停。
- 池内定高用 **DVL altitude**(非压力深度, 不受水面基准漂移); 航向用 **ATTITUDE.yaw**(DVL 的 yaw 无罗盘会漂)。
- **设定值节流**: 实测 surge ~0.06 m/s < v_cruise 0.15 → 不节流则设定值超前 1.18m、u_x 长期饱和退化成开关控制; 节流后滞后 <0.1m。

**已修复(均 commit)**: 深度源污染(VFR_HUD 恒0 混入 parse_depth) · DVL 假丢底锁(is_fresh 被坏帧误判) · Cockpit 手柄抢 MANUAL_CONTROL(加竞争源告警) · 航向跳变→急旋(陀螺交叉校验剔除) · 航向参考系平移→平移目标而非打舵追 · heading 积分项 ki(抗垂直-偏航耦合恒定扰动) · go_forward `--hold-timeout` 40s。

**待办**: `waypoint/RESULTS.md` §3 现场数据表仍待回填(实测下潜/距离/航向/深度数值)。

**另一条线 direct_thruster**(独立, 见 `direct_thruster/HANDOFF.md`): 修改+编译 ArduSub 独立控 8 桨。进行到 **M1**(WSL2 Ubuntu-22.04 + ArduSub-4.1.2 clone[与实机 hash 一致] + 手动装构建依赖), 卡点=验证 GCC10.2 交叉工具链 → 下一步 M2 编译 vanilla。刷机须排在 waypoint 收尾之后。

### direct_thruster M2 编译门槛 — ✅ 通过 (2026-09-17, dev 侧)
- `./waf configure --board navigator --toolchain $ARM_TC/bin/arm-none-linux-gnueabihf` + `./waf sub` 编译成功。
- 产物 `build/navigator/bin/ardusub` 1.9 MiB;`file` = **ELF 32-bit LSB ARM EABI5**,加载器 `/lib/ld-linux-armhf.so.3`(armhf,符合 Bullseye BlueOS)。
- 结论:**WSL + ArduSub-4.1.2 源码 + GCC10.2 工具链 + Navigator 构建链路全部可用**。
- **M2 尚未完成的另一半 = 装机验证**(把这个 vanilla 刷上 ROV 证明能启动)。受"刷机排在 waypoint 之后 + 用户显式同意 + 物理隔离推进器"门控。
- M3(改 C++)可在 dev 侧先做、不需刷机;但**不得在 vanilla 装机验证通过前刷改过的固件**。
