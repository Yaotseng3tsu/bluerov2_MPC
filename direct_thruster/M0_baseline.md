# M0 基线备份（direct_thruster）

> 记录实机当前固件/参数基线，作为 fork 的起点与回退依据。采集日：2026-09-16（只读，未改动任何参数）。

## 固件 / 平台
| 项 | 值 |
|---|---|
| Autopilot 固件 | **ArduSub 4.1.2 (STABLE / official)**，git `2dd0bb7d` |
| Board | **Navigator**（Blue Robotics），mavlink platform `navigator`，Vehicle = Submarine |
| BlueOS | **1.4.2**（running，`15af2185`；另有 factory `15af2185`、1.4.1 `a0e86a01`） |
| Phase1 checkout tag | **`ArduSub-4.1.2`** |

> 版本较老（~2022）。构建端：BlueOS 1.4.x 基于 **Bullseye** → 交叉工具链用 **GCC 10.2**；
> 老 4.1 分支建议构建主机 **Ubuntu 20.04**（waf/依赖更匹配，M1 再最终确认）。

## 推进器映射（备份）
- `FRAME_CONFIG = 2` → **Vectored 6DOF（BlueROV2 Heavy，8 推进器）**；与干测确认的标准 Heavy 分配一致。
- `FRAME_CLASS / FRAME_TYPE = None`（ArduSub 用 FRAME_CONFIG，属正常）。

| 通道 | FUNCTION | REVERSED | MIN | MAX | TRIM |
|---|---|---|---|---|---|
| SERVO1 | Motor1 | 0 | 1100 | 1900 | 1500 |
| SERVO2 | Motor2 | 0 | 1100 | 1900 | 1500 |
| SERVO3 | (读包丢失,几乎必为 Motor3) | 0 | 1100 | 1900 | 1500 |
| SERVO4 | Motor4 | 0 | 1100 | 1900 | 1500 |
| SERVO5 | Motor5 | 0 | 1100 | 1900 | 1500 |
| SERVO6 | Motor6 | 0 | 1100 | 1900 | 1500 |
| SERVO7 | Motor7 | 0 | 1100 | 1900 | 1500 |
| SERVO8 | Motor8 | 0 | 1100 | 1900 | 1500 |

> 即 **8 路输出通道 1–8 = Motor1–8**，未反向，PWM 1100–1900、中位 1500。Phase3 改 C++ 时"external[0..7] → Motor1..8"按此对应。

## 待补（M0-b，BlueOS 网页）
- [ ] `Autopilot Parameters` → 导出完整参数文件（整机备份，存档路径：__________）
- [x] `Autopilot Firmware` 有 **RESTORE DEFAULT FIRMWARE**（回退路径确认存在）
- [ ] 记 BlueOS 当前镜像可回退（Local Versions：1.4.2 running / factory / 1.4.1）

## 决策点
- [ ] **匹配当前 4.1.2 fork**（推荐，waypoint 已在此版验证、最小扰动）
      vs **先升级 ArduSub/BlueOS 再从新版 fork**（代码更新、构建更易，但改动现有可用环境）。默认：匹配 4.1.2。
