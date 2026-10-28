# NOTES

## 设计决策

一条连接、一个读线程。MAVLink 是一条字节流，`recv_match()` 会把消息从流里取走。
如果任务线程和电池监控线程各自 `recv_match()`，两边会互相抢消息（监控线程可能吃掉
任务线程正在等的 `COMMAND_ACK`）。所以 `Drone` 里只有一个后台读线程，它把最新遥测
写进共享快照，其余线程只读快照，用 `Condition` 唤醒。这同时满足 Part 3
"共享一条 MAVLink 连接"的要求。

命令双重确认。`COMMAND_ACK` 只代表飞控收到了命令，不代表事情发生了。所以
`set_mode` / `arm` 都是"回执 accepted 且遥测状态改变"才算成功；起飞是等到
`relative_alt` 真的进入 0.5 m 才返回；参数写入是等到 `PARAM_VALUE` 回读到目标值才算数。

没有盲等。所有等待都是"轮询条件 + 截止时间"。循环里的 `time.sleep(0.1)` 只是
采样间隔，不是"睡够就一定成功"。每一步失败都抛 `DroneError`，由入口统一转成
清晰错误信息 + 非零退出码，不会挂死也不会抛 traceback。

Part 3 的并发选了 `threading.Thread` + `Event`。 理由：pymavlink 是同步阻塞 API，
换 asyncio 得把整个收包层包一层 executor，收益不大；而单循环状态机虽然最省线程，
但"任务推进"和"安全检查"会耦合在同一个循环里，安全逻辑容易被任务逻辑的等待阻塞
（比如等待到点期间电压检查就停摆了）。线程方案让安全检查以固定频率独立运行，
代价只有一个 `Event` 和一把发送锁。

正方形几何。用起点 + 北/东偏移算四个角点，最后一个角点就是起点，飞完自然
回到出发点。GUIDED 位置目标以 1 Hz 重发，防止单帧丢失；每点判定用 Haversine
水平距离 ≤ 2 m。

遥测 10 Hz。 5 m/s 飞行时每个采样点只移动 0.5 m，2 m 的判定阈值才够准。



## 意外的发现

- EKF flags 在开机后约 40 秒才从 `0x00a7` 变成 `0x033f`（相差 `EKF_POS_HORIZ_ABS`
  和 `EKF_POS_VERT_ABS` 两位），和题面说的 40–60 秒一致；只看 `gps_fix=6` 会过早
  放行。
- 起飞目标 15 m，实际稳定在 14.56 m 就满足了 ±0.5 m —— ArduCopter 在 GUIDED 起飞
  后不追求精确悬停到设定值。
- `SIM_BATT_VOLTAGE` 写入后，下一个 `SYS_STATUS` 周期（0.1 s 内）电压就掉到 10.50 V，
  失保监控检测 + 下发 RTL 总共花了约 0.2 秒。
- RTL 落地后飞控会等约 10 秒才自动上锁，`wait_landed` 必须同时等"高度 < 0.5 m"
  和"disarmed"，只等高度会提前退出。
- Python 的 `threading.Thread` 内部有 `_stop()` 方法，子类里把 `self._stop` 设成
  `Event` 会让 `join()` 崩掉（`TypeError: 'Event' object is not callable`），属性名要避开。
