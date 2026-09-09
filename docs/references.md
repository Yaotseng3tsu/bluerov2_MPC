# 参考资料索引

本地文件位于 `C:\Users\yaots\OneDrive\Desktop\BlueROV MPC\References\`(未纳入 git,体积大)。

| 文件 | 内容 | 在本项目中的作用 |
|---|---|---|
| `Stationkeep Discussion.docx` | DVL + ArduSub Position Hold 原理、官方 PSC 起始参数(POSXY_P=2.0, VELXY_P=5.0, VELXY_I=0.5, VELXY_D=0.8, POSZ_P=1.0, VELZ_P=5.0)、单维低速测试建议、yaw/安装方向的三大坑 | **选深度作为首维的依据**;PID 结构与调参直觉 |
| `MPC_control_for_the_BlueROV2_Theory_and_Implementation.pdf` | AAU 2020 硕士论文:6-DOF 建模、Kalman 滤波、CasADi/IPOPT NMPC、与 PID/LQR 对比 | **建模与 MPC 实现蓝本**(P3/P5) |
| `2506.21063v1.pdf` | arXiv,R4 / BlueOS 相关 | R4 平台背景参考 |
| [HKPolyU-UAV/bluerov2](https://github.com/HKPolyU-UAV/bluerov2) | ROS + ACADOS/CasADi 的 MPC 实现 | 结构参考;本项目做极简 Python 版,不跑其 ROS 环境 |

## 关键技术备注
- 官方 Station Keeping = DVL 观测 + ArduSub 内置 Position Hold(PID),非 DVL 直接控推进器。
- 深度反馈来自压力传感器,不依赖 DVL/yaw → 最适合作第一个闭环维度。
- MANUAL_CONTROL 的 `z`(垂直)中位与正负号需实测确认(P1)。
- 现有 `C:\bluerov2_rl_logger` 仓库已验证 pymavlink over BlueOS(UDP 14560)链路可用,可复用其收包/时间对齐经验。
