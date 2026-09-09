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
- git commit: (初始提交)
