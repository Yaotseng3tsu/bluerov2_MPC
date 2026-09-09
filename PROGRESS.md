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
- git commit: (P0 提交)
