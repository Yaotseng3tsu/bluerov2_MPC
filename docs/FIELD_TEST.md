# 现场实测手册 — Day 1(P0 连接 → P1 让推进器动起来)

> 目标:出水/台架干测环境下,**代码连上真机 → 确认深度遥测 → 解锁 → 逐轴标定方向/中位 → 用归一化指令驱动推进器**。
> 全程在监管下逐步进行,每步有通过标准和记录点。**不入水**。

---

## 0. 安全总则(务必先读)

- 🔴 **桨叶危险**:解锁后推进器会转。人员、线缆、衣物远离桨叶。
- 🔴 **出水/固定**:ROV 固定在台架或垫高,推进器悬空。T200 在空气中**短时**运行可以,但**不要长时间空转**(轴承靠水润滑)——每次脉冲 ≤1s,间隔散热。
- 🔴 **一人负责急停**:一人手放 Ctrl+C / 电源开关,发现异常立即停。
- 🟡 **先确认 ArduSub 失联失效**:见第 2 节。硬崩溃时靠它兜底。
- 🟡 **低幅值起步**:`U_MAX=0.3`,标定脉冲 `0.2`。确认无误再加大。
- 每一步**先看、后动**;进入下一步前和监管确认。

---

## 1. 前置准备

### 1.1 BlueOS / 网络
1. ROV 通电,BlueOS 启动,Cockpit 能看到状态/视频/遥测(先用官方地面站确认整机正常)。
2. BlueOS → **Pirate Mode** → **MAVLink Endpoints**,新建:
   - 类型 `UDP Client`,IP = **本上位机 IP**,端口 **`14550`**。
   - (与现有 logger 的 14560、Cockpit 的端口区分,避免抢包。)
3. 若探测不到消息,把路由器切到 **MAVLinkServer** 模式。
4. **Windows 防火墙**放行 Python 接收 UDP `14550`。
5. 记录:上位机 IP = ______,ROV IP = ______。

### 1.2 上位机环境
```powershell
cd C:\bluerov2_mpc
.\.venv\Scripts\python.exe -c "import pymavlink, yaml, numpy; print('env OK')"
```

---

## 2. 关键前提:确认 ArduSub 失联失效(硬崩溃兜底)

我们的代码在**正常退出 / Ctrl+C** 时会自动回中位 + 上锁。但若进程**硬崩溃/断网**,来不及发上锁命令 —— 这时只能靠 ArduSub 自身的失效保护。**下水前必须确认**(今天干测也建议先看):

- 在 BlueOS/参数里确认 **Pilot input / GCS failsafe** 行为(停发 MANUAL_CONTROL 或 GCS 心跳后,ArduSub 会中和输出 / disarm)。
- 记录:失联后行为 = ______,超时 ≈ ______ s。

> 说明:仿真里我们用 `cmd-timeout≈1.5s` 模拟了这一行为并验证过;真机的实际值以此处确认为准。

---

## 3. 步骤 P0 — 连接自检(只读,不解锁)

```powershell
.\.venv\Scripts\python.exe -m src.link --check --seconds 30
```
测试中**用手上下移动/倾斜 ROV**(或改变深度计读数),观察深度变化。

**通过标准**:
- 打印 `heartbeat OK`,arm 状态 = DISARMED。
- 小结显示 heartbeat 与深度都有,深度 Hz ≥ 3(闭环最好 ≥ CTRL_HZ=10)。
- 深度符号:**下潜 → 深度值增大**(若相反,记下来,标定/配置里处理)。

**记录**:深度源 = ______,实测 Hz = ______,符号是否正确 = ______。
**排查**:见第 7 节。

---

## 4. 步骤 P1-a(可选)— 确认"未解锁不动"

先不解锁,发一个下潜指令,确认推进器**不转**(验证安全默认):
```powershell
.\.venv\Scripts\python.exe -m src.pseudo_stick --z 0.2 --seconds 3
```
**通过标准**:推进器不转(未 arm)。脚本结束打印"已发送中位并关闭"。

---

## 5. 步骤 P1-b — 解锁 + 方向/中位标定(核心)

> ⚠ 此步推进器会转。再次确认出水、远离桨叶、一人守急停。

```powershell
.\.venv\Scripts\python.exe -m src.calibrate --axes xyzr --pulse 0.2 --dur 0.8
```
交互流程(逐轴):
1. 安全确认输入 `yes` → 设 MANUAL → 解锁(打印"已解锁 ARMED")。
2. 每轴:回车 → 发 `+0.2` 脉冲 0.8s → **观察运动方向** → 答 `y`(与期望一致)/`n`(反向)/`s`(跳过)。
   - 期望正方向:`x`=前进,`y`=右移,`z`=下潜,`r`=右转。
3. 结束自动:回中位 + **自动上锁**,并把 `sign_*` 写回 `config/vehicle.yaml`。

**z 中位确认**:发 `z` 脉冲前后,ROV 静止时垂直推进器应停转;若中位不停,记录并调 `z_neutral`。

**通过标准**:四轴符号确定并写入 config;每次脉冲后能安全停止;退出后确认已 DISARMED。
**记录**:sign_x/y/z/r = ______,z_neutral = ______。

**若解锁失败**(ACK 报错):见第 7 节;可 `--force-arm`,或先在 Cockpit 解锁再跑(那样本脚本不负责上锁,你手动上锁)。

---

## 6. 步骤 P1-c — 最低目标演示(代码让推进器动起来)

标定完成后,用归一化指令驱动指定推进器(**最低目标达成点**):
```powershell
# 单轴短促驱动(会自动解锁并在结束/中断时自动上锁)
.\.venv\Scripts\python.exe -m src.pseudo_stick --z 0.2 --seconds 2 --arm
# 前进方向:
.\.venv\Scripts\python.exe -m src.pseudo_stick --x 0.2 --seconds 2 --arm
```
**通过标准**:指定推进器按归一化指令正/反转;`Ctrl+C` 能立即中位并上锁;结束后 DISARMED。
✅ 至此"通过代码给出归一化运动指令让机器人动起来"达成。

---

## 7. 排查速查

| 现象 | 可能原因 / 处理 |
|---|---|
| 无 heartbeat | 端点 IP/端口错;防火墙没放行 14550;换 MAVLinkServer 模式;确认 Cockpit 能连 |
| 有 heartbeat 无深度 | 深度源不对;`config` 里 `depth_message` 改 `SCALED_PRESSURE2` 再试;提高消息频率 |
| 深度符号反 | 下潜时深度变小 → 记录;闭环阶段在解析/配置里翻符号 |
| 解锁 ACK 失败 | 预检未过(电压/传感器/水检等);台架干测可 `--force-arm`;或 Cockpit 手动解锁 |
| 推进器方向全反 | 标定时答 `n`,`sign_*` 会记 -1 |
| 深度 Hz 太低 | 提高 `SET_MESSAGE_INTERVAL`;确认路由不丢包 |
| 进程崩溃后仍解锁 | 靠 ArduSub 失联失效(第 2 节);手动断电/Cockpit 上锁 |

---

## 8. 收尾

1. 确认 **DISARMED**(Cockpit 或再跑一次 `link --check` 看 arm 状态)。
2. 断开推进器电源 / 电池。
3. 把记录填进 [`PROGRESS.md`](../PROGRESS.md)(深度源/Hz/符号/z_neutral/失联行为/遇到的问题)。
4. 提交:`git add -A && git commit -m "P0/P1 field test: <结果摘要>"`,`git push`。
5. 和监管确认是否进入 P3(系统辨识,需入水)。

---

## 附:今天会用到的命令一览
```powershell
cd C:\bluerov2_mpc
.\.venv\Scripts\python.exe -m src.link --check --seconds 30              # P0 自检
.\.venv\Scripts\python.exe -m src.pseudo_stick --z 0.2 --seconds 3       # P1-a 未解锁不动
.\.venv\Scripts\python.exe -m src.calibrate --axes xyzr                  # P1-b 标定(解锁)
.\.venv\Scripts\python.exe -m src.pseudo_stick --z 0.2 --seconds 2 --arm # P1-c 驱动
# 任意时刻 Ctrl+C = 立即中位 + 自动上锁
```
