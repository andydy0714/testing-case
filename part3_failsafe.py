#!/usr/bin/env python3
"""Part 3 — 正方形任务 + 电池失保监控（并发）。

任务流程：
    起飞 → 飞正方形 → 第 20 秒注入故障（SIM_BATT_VOLTAGE=10.5）
         → 电压跌破 11.0 V → 监控线程置位并 RTL
         → 任务线程立刻停止发 goto → 等待返航落地 → 打印完整日志
"""

from __future__ import annotations

import sys
import threading
import time
from datetime import datetime

from starter.drone import Drone, DroneError, distance_m, require_position, square_corners

SIDE_M = 80.0
ALT_M = 15.0
CORNER_TOL_M = 2.0
CORNER_TIMEOUT_S = 120.0

THRESHOLD_V = 11.0       # 低于这个电压就算失保
INJECT_VOLTAGE_V = 10.5  # 注入的模拟电池电压
TRIGGER_AFTER_S = 20.0   # 任务开始多久后注入故障


def log(message: str) -> None:
    """带毫秒时间戳的日志。"""
    stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{stamp}] {message}", flush=True)


class BatteryFailsafe(threading.Thread):
    """盯着 SYS_STATUS 电压；跌破阈值就中止任务并 RTL。"""

    def __init__(self, drone: Drone, threshold_v: float, abort_event: threading.Event) -> None:
        super().__init__(name="battery-failsafe", daemon=True)
        self.drone = drone
        self.threshold_v = threshold_v
        self.abort_event = abort_event
        # 注意：不能叫 self._stop —— threading.Thread 内部有同名方法 _stop()，
        # 覆盖掉之后 join() 会抛 TypeError: 'Event' object is not callable。
        self._stop_event = threading.Event()
        self.reason: str | None = None

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        while not self._stop_event.is_set():
            st = self.drone.state
            # 只看"已解锁且在飞"的状态：落地后电压低是正常的
            if st.armed and st.battery_v is not None and st.battery_v < self.threshold_v:
                self._abort(st.battery_v)
                return
            self._stop_event.wait(0.1)  # 采样间隔

    def _abort(self, voltage: float) -> None:
        if self.abort_event.is_set():
            return
        self.reason = f"电池电压 {voltage:.2f} V 低于阈值 {self.threshold_v:.1f} V"
        log(f"失保触发：{self.reason}")
        self.abort_event.set()  # 先置位：任务线程立刻停止发 goto
        log("已停止下发位置目标，命令 RTL")
        try:
            self.drone.rtl(timeout=10.0)
            log("飞控已确认进入 RTL 模式")
        except DroneError as exc:
            log(f"RTL 命令失败：{exc}")


def fly_square(
    drone: Drone,
    corners: list[tuple[float, float]],
    abort_event: threading.Event,
) -> str:
    """飞正方形。返回 "aborted" 或 "completed"。"""
    mission_start = time.monotonic()
    injected = False

    for index, (lat, lon) in enumerate(corners, start=1):
        log(f"飞向角点 {index}/4：{lat:.7f}, {lon:.7f}")
        deadline = time.monotonic() + CORNER_TIMEOUT_S
        last_send = 0.0
        last_print = 0.0

        while True:
            if abort_event.is_set():
                log(f"任务中止：放弃剩余角点（角点 {index} 未到达）")
                return "aborted"

            now = time.monotonic()

            # —— 故障注入：任务进行到 TRIGGER_AFTER_S 秒时把模拟电池改成 10.5 V
            if not injected and now - mission_start >= TRIGGER_AFTER_S:
                injected = True
                log(f"注入故障：SIM_BATT_VOLTAGE = {INJECT_VOLTAGE_V}")
                try:
                    readback = drone.param_set("SIM_BATT_VOLTAGE", INJECT_VOLTAGE_V, timeout=10.0)
                    log(f"参数已确认写入：SIM_BATT_VOLTAGE 回读 {readback:.2f}")
                except DroneError as exc:
                    log(f"参数注入失败：{exc}")

            # 1 Hz 重发位置目标
            if now - last_send >= 1.0:
                drone.goto(lat, lon, ALT_M)
                last_send = now

            st = drone.state
            dist = None
            if st.lat is not None:
                dist = distance_m(st.lat, st.lon, lat, lon)
                if now - last_print >= 1.0:
                    last_print = now
                    voltage = f"{st.battery_v:.2f} V" if st.battery_v is not None else "?"
                    log(
                        f"  角点{index}: 距目标 {dist:6.1f} m, "
                        f"高度 {st.alt_rel:.1f} m, 电压 {voltage}"
                    )
                if dist <= CORNER_TOL_M:
                    log(f"到达角点 {index}（{dist:.2f} m）")
                    break

            if now >= deadline:
                raise DroneError(f"超时 {CORNER_TIMEOUT_S:.0f}s：角点 {index} 未到达")

            time.sleep(0.1)  # 轮询间隔

    log("正方形飞完，回到起点")
    return "completed"


def main() -> int:
    drone = None
    monitor = None
    abort_event = threading.Event()

    try:
        log("连接 SITL ...")
        drone = Drone()

        log("等待飞控就绪（EKF/GPS 收敛）")
        drone.wait_ready(timeout=120.0)

        log("切 GUIDED、解锁、起飞")
        drone.set_mode("GUIDED")
        drone.arm()
        drone.takeoff(ALT_M, timeout=60.0)

        lat0, lon0 = require_position(drone)
        corners = square_corners(lat0, lon0, SIDE_M)
        log(f"正方形任务开始：边长 {SIDE_M:.0f} m，起点 {lat0:.7f}, {lon0:.7f}")

        # 监控线程和任务线程并行，共用一条连接
        monitor = BatteryFailsafe(drone, THRESHOLD_V, abort_event)
        monitor.start()
        log(f"失保监控已启动：阈值 {THRESHOLD_V:.1f} V，采样 10 Hz")

        outcome = fly_square(drone, corners, abort_event)

        if outcome == "completed":
            log("任务正常完成，命令 RTL 返航")
            try:
                drone.rtl()
            except DroneError as exc:
                log(f"RTL 命令失败：{exc}")
        else:
            # 监控线程已经发过 RTL；这里再确认一次模式，防止命令丢失
            if drone.state.mode != "RTL":
                try:
                    drone.rtl()
                except DroneError as exc:
                    log(f"补发 RTL 失败：{exc}")

        log("等待返航落地并自动上锁 ...")
        drone.wait_landed(timeout=240.0)

        reason = monitor.reason if monitor.reason else "任务正常完成"
        log(f"任务结束：{reason}；已落地并上锁")
        return 0

    except DroneError as exc:
        log(f"错误：{exc}")
        return 1
    except KeyboardInterrupt:
        log("已手动中断")
        return 130
    finally:
        if monitor is not None:
            monitor.stop()
            monitor.join(timeout=2.0)
        if drone is not None:
            drone.close()


if __name__ == "__main__":
    sys.exit(main())
