# 交付包打包清单 (从 2026-09 旧版 dist/ 考古得出)

> `dist/` **没有打包脚本**, 是手工打的包。2026-10-06 删除旧 dist/ 前把这三份
> 手写文档和下面的文件清单留存下来, 供任务 ③ 重新打包时用。
> 旧包的数据是 **2026-09-17 受手柄冲突污染的那一批**, 重新打包时数据部分要换成干净基线。

## 本目录三份文档的来源
| 本文件 | 原路径 |
|---|---|
| `README_package_full.md` | `dist/bluerov2_dvl_altitude_forward/README.md` (238 行) |
| `README_package_code_only.md` | `dist/bluerov2_dvl_altitude_forward_code_only/README.md` (230 行) |
| `README_DATA.md` | `dist/bluerov2_dvl_altitude_forward/recorded_data/README_DATA.md` (97 行) —— **数据可信度台账, 逐文件判定 9/17 哪些可信** |

## 完整包 bluerov2_dvl_altitude_forward (79 文件)
```
README.md
config/vehicle.yaml
recorded_data/README_DATA.md
recorded_data/figures/real/altitude_hold_run.png
recorded_data/figures/sitl/heading_hold_selftest.png
recorded_data/real_2026-09-17/altitude_hold/*.csv
recorded_data/real_2026-09-17/dvl_raw/*.csv
recorded_data/real_2026-09-17/go_forward/*.csv
recorded_data/sitl/altitude_hold/*.csv
recorded_data/sitl/go_forward/*.csv
requirements.txt
src/__init__.py
src/pid.py
src/pseudo_stick.py
waypoint/__init__.py
waypoint/altitude_hold.py
waypoint/data/.gitkeep
waypoint/data/dvl_data/.gitkeep
waypoint/dvl_dashboard.py
waypoint/dvl_stream.py
waypoint/dvl_traj_log.py
waypoint/go_forward.py
waypoint/heading_hold.py
waypoint/plot_dvl_traj.py
```

## 纯代码包 bluerov2_dvl_altitude_forward_code_only (16 文件)
```
README.md
config/vehicle.yaml
requirements.txt
src/__init__.py
src/pid.py
src/pseudo_stick.py
waypoint/__init__.py
waypoint/altitude_hold.py
waypoint/data/.gitkeep
waypoint/data/dvl_data/.gitkeep
waypoint/dvl_dashboard.py
waypoint/dvl_stream.py
waypoint/dvl_traj_log.py
waypoint/go_forward.py
waypoint/heading_hold.py
waypoint/plot_dvl_traj.py
```
