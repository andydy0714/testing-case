"""无人机控制器：MAVLink 连接、遥测缓存、常用飞行动作。"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Optional

from pymavlink import mavutil


class DroneError(Exception):
    """命令被拒绝或步骤超时时抛出。"""


# 常量

EARTH_RADIUS_M = 6378137.0

# EKF_STATUS_REPORT.flags 的位定义（ArduPilot）
EKF_ATTITUDE = 1
EKF_VELOCITY_HORIZ = 2
EKF_POS_HORIZ_ABS = 16
EKF_POS_VERT_ABS = 32
EKF_UNINITIALIZED = 1024

# Guided 起飞需要：姿态 + 水平速度 + 水平绝对位置 + 垂直绝对位置
EKF_READY_FLAGS = EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_ABS | EKF_POS_VERT_ABS

# SET_POSITION_TARGET_GLOBAL_INT 的 type_mask：只使用位置，
# 忽略速度(bit3-5)/加速度(bit6-8)/力(bit9)/偏航(bit10)/偏航角速度(bit11)
TYPE_MASK_POSITION_ONLY = 0b0000_1111_1111_1000  # = 4088

# COMMAND_ACK.result 的可读名字，报错时好定位
MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}


# 遥测快照

@dataclass
class Telemetry:
    """读线程写、业务线程读的最新状态快照。"""

    mode: str = "?"
    armed: bool = False
    lat: Optional[float] = None          # 度
    lon: Optional[float] = None          # 度
    alt_msl: Optional[float] = None      # 米（海拔）
    alt_rel: Optional[float] = None      # 米（相对 home）
    battery_v: Optional[float] = None    # 伏
    ekf_flags: int = 0
    gps_fix_type: int = 0
    last_statustext: str = ""

# 几何工具

def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """两点的水平距离（Haversine，米）。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def offset_position(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """从 (lat, lon) 往北 north_m 米、往东 east_m 米后的坐标。

    小范围（几百米）内用平面近似足够，误差远小于 2 m 的判定阈值。
    """
    dlat = north_m / EARTH_RADIUS_M * 180.0 / math.pi
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat))) * 180.0 / math.pi
    return lat + dlat, lon + dlon


def square_corners(lat: float, lon: float, side_m: float) -> list[tuple[float, float]]:
    """以 (lat, lon) 为起点，逆时针（北→东→南→西）绕一圈的四个角点。

    最后一个角点就是起点，所以飞完会自动回到出发点。
    """
    north_east = offset_position(lat, lon, side_m, side_m)
    north = offset_position(lat, lon, side_m, 0.0)
    east = offset_position(lat, lon, 0.0, side_m)
    return [north, north_east, east, (lat, lon)]


# 控制器

class Drone:
    """MAVLink 客户端。/ 用法::

        with Drone() as d:
            d.wait_ready()
            d.set_mode("GUIDED")
            d.arm()
            d.takeoff(15)
    """

    POLL_S = 0.1          # 轮询间隔
    GOTO_RESEND_S = 1.0   # GUIDED 位置目标的重发周期
    LINK_TIMEOUT_S = 5.0  # 多久收不到任何消息就认为链路挂了

    def __init__(
        self,
        connection_string: str = "tcp:127.0.0.1:5760",
        heartbeat_timeout: float = 30.0,
        stream_rate_hz: int = 10,
    ) -> None:
        # 连接失败（SITL 没起）时 pymavlink 直接抛 ConnectionRefusedError，
        # 这里统一转成 DroneError，保证上层只处理一种异常、不会漏出 traceback。
        try:
            self.conn = mavutil.mavlink_connection(connection_string)
        except OSError as exc:
            raise DroneError(
                f"连不上 {connection_string} —— SITL 是不是没启动？（docker compose up -d）"
                f"（{exc}）"
            ) from None

        if self.conn.wait_heartbeat(timeout=heartbeat_timeout) is None:
            self.conn.close()
            raise DroneError(
                f"{heartbeat_timeout:.0f} 秒内收不到心跳 —— SITL 是不是卡住了？（docker compose restart）"
            )

        self.target_system = self.conn.target_system
        self.target_component = self.conn.target_component

        # 请求全部数据流。10 Hz 是为了 Part 2 的 2 m 判据足够灵敏
        # （5 m/s 时每个采样点只走 0.5 m）。
        self.conn.mav.request_data_stream_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            stream_rate_hz,
            1,
        )

        self._lock = threading.RLock()               # 保护状态快照
        self._cv = threading.Condition(self._lock)   # 状态变化时唤醒等待者
        self._send_lock = threading.Lock()           # 串行化所有发送

        self._state = Telemetry()
        self._acks: dict[int, int] = {}              # command -> result
        self._params: dict[str, float] = {}
        self._home: Optional[tuple[float, float, float]] = None

        self._link_error: Optional[str] = None
        self._last_msg_time = time.monotonic()
        self._stop = threading.Event()

        self._reader = threading.Thread(target=self._reader_loop, name="mavlink-reader", daemon=True)
        self._reader.start()

    # 生命周期

    def __enter__(self) -> "Drone":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    def close(self) -> None:
        self._stop.set()
        with self._cv:
            self._cv.notify_all()
        if self._reader.is_alive():
            self._reader.join(timeout=2.0)
        try:
            self.conn.close()
        except Exception:
            pass

    @property
    def state(self) -> Telemetry:
        """当前遥测快照（拷贝，调用方随便用）。"""
        with self._lock:
            return replace(self._state)

    @property
    def home(self) -> Optional[tuple[float, float, float]]:
        with self._lock:
            return self._home

    # 读线程 

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception as exc:  # 连接断开等
                with self._cv:
                    self._link_error = f"读取 MAVLink 失败：{exc}"
                    self._cv.notify_all()
                return
            if msg is None:
                continue
            self._handle_message(msg)

    def _handle_message(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "BAD_DATA":          # 解析失败的分片，丢掉即可
            return

        with self._cv:
            self._last_msg_time = time.monotonic()

            if mtype == "HEARTBEAT" and msg.get_srcSystem() == self.target_system:
                self._state.mode = mavutil.mode_string_v10(msg)
                self._state.armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

            elif mtype == "GLOBAL_POSITION_INT":
                # lat/lon 是「度 × 1e7」的整数，高度是毫米
                self._state.lat = msg.lat / 1e7 if msg.lat != 0 else self._state.lat
                self._state.lon = msg.lon / 1e7 if msg.lon != 0 else self._state.lon
                self._state.alt_msl = msg.alt / 1000.0
                self._state.alt_rel = msg.relative_alt / 1000.0

            elif mtype == "SYS_STATUS":
                # voltage_battery 单位是 mV；0xFFFF 表示"未知"
                if msg.voltage_battery != 0xFFFF:
                    self._state.battery_v = msg.voltage_battery / 1000.0

            elif mtype == "EKF_STATUS_REPORT":
                self._state.ekf_flags = msg.flags

            elif mtype == "GPS_RAW_INT":
                self._state.gps_fix_type = msg.fix_type

            elif mtype == "STATUSTEXT":
                self._state.last_statustext = _to_str(msg.text).rstrip("\x00")

            elif mtype == "COMMAND_ACK":
                self._acks[msg.command] = msg.result

            elif mtype == "PARAM_VALUE":
                self._params[_to_str(msg.param_id).rstrip("\x00")] = msg.param_value

            elif mtype == "HOME_POSITION":
                self._home = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)

            self._cv.notify_all()

    # 内部工具

    def _raise_if_link_dead(self) -> None:
        """链路异常就抛错，避免业务代码无限等下去。（调用时须持锁）"""
        if self._link_error:
            raise DroneError(self._link_error)
        if time.monotonic() - self._last_msg_time > self.LINK_TIMEOUT_S:
            raise DroneError(f"遥测中断超过 {self.LINK_TIMEOUT_S:.0f} 秒")

    def _wait(
        self,
        predicate: Callable[[], bool],
        timeout: float,
        what: str,
        progress: Optional[Callable[[], None]] = None,
        progress_interval: float = 1.0,
    ) -> None:
        """等到 predicate() 为真；超时抛 DroneError。

        predicate 在持锁状态下执行，可以直接读 self._state。
        """
        deadline = time.monotonic() + timeout
        next_progress = time.monotonic() + progress_interval

        with self._cv:
            while True:
                self._raise_if_link_dead()
                if predicate():
                    return
                now = time.monotonic()
                if now >= deadline:
                    raise DroneError(
                        f"超时 {timeout:.0f}s：{what}"
                        + (f"（飞控最后消息：{self._state.last_statustext}）"
                           if self._state.last_statustext else "")
                    )
                if progress is not None and now >= next_progress:
                    progress()
                    next_progress = now + progress_interval
                self._cv.wait(min(0.2, max(0.0, deadline - now)))

    def _command(self, command: int, *params: float) -> None:
        """发送 COMMAND_LONG（7 个参数，缺省补 0）。"""
        p = list(params) + [0.0] * (7 - len(params))
        with self._send_lock:
            self.conn.mav.command_long_send(
                self.target_system, self.target_component, command, 0,
                p[0], p[1], p[2], p[3], p[4], p[5], p[6],
            )

    def _clear_ack(self, command: int) -> None:
        with self._lock:
            self._acks.pop(command, None)

    def _ack_error(self, command: int) -> Optional[str]:
        """如果收到过拒绝回执，返回可读的原因。（调用时须持锁）"""
        result = self._acks.get(command)
        if result is None or result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return None
        return MAV_RESULT_NAMES.get(result, f"result={result}")

    # 动作

    def wait_ready(self, timeout: float = 120.0) -> None:
        """等到 EKF/GPS 收敛、可以安全起飞。

        SITL 开机后要 40–60 s 等模拟 GPS 定位和 EKF 收敛，
        提前解锁会被飞控拒绝。
        """
        start = time.monotonic()

        def progress() -> None:
            st = self._state
            print(
                f"  等待 EKF 收敛 {time.monotonic() - start:4.0f}s | "
                f"flags={st.ekf_flags:#06x} gps_fix={st.gps_fix_type} alt={st.alt_rel}",
                flush=True,
            )

        self._wait(
            lambda: (self._state.ekf_flags & EKF_READY_FLAGS) == EKF_READY_FLAGS
                    and self._state.alt_rel is not None,
            timeout,
            "EKF/GPS 收敛（通常需要 40–60 秒）",
            progress=progress,
            progress_interval=5.0,
        )
        print(f"  飞控就绪：EKF flags={self._state.ekf_flags:#06x}", flush=True)

    def set_mode(self, mode_name: str, timeout: float = 10.0) -> None:
        """切换飞行模式，并确认模式真的切过去了（看心跳，不只看回执）。"""
        mapping = self.conn.mode_mapping() or {}
        if mode_name not in mapping:
            raise DroneError(f"未知模式 {mode_name!r}，可选：{sorted(mapping)}")
        mode_id = mapping[mode_name]

        self._clear_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
        self._command(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            float(mode_id),
        )

        def ok() -> bool:
            reason = self._ack_error(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
            if reason:
                raise DroneError(f"切换 {mode_name} 被拒绝：{reason}")
            return self._state.mode == mode_name

        self._wait(ok, timeout, f"切换到 {mode_name} 模式")
        print(f"  模式已确认为 {mode_name}", flush=True)

    def arm(self, timeout: float = 30.0) -> None:
        """解锁电机。回执 accepted + 遥测 armed 双重确认。"""
        if self.state.armed:
            print("  已经是解锁状态", flush=True)
            return

        self._clear_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        # param1 = 1 表示 arm
        self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)

        def ok() -> bool:
            reason = self._ack_error(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            if reason:
                raise DroneError(
                    f"解锁被拒绝：{reason}"
                    + (f"（飞控消息：{self._state.last_statustext}）"
                       if self._state.last_statustext else "")
                )
            return self._state.armed

        self._wait(ok, timeout, "解锁电机")
        print("  电机已解锁（回执 + 遥测一致）", flush=True)

    def takeoff(self, altitude_m: float, timeout: float = 60.0, tolerance: float = 0.5) -> None:
        """起飞到相对 home 的 altitude_m 米，阻塞到误差 ≤ tolerance。"""
        if not self.state.armed:
            raise DroneError("电机未解锁，不能起飞")

        self._clear_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        # GUIDED 模式下 param7 = 相对 home 的目标高度（米）
        self._command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, altitude_m)

        deadline = time.monotonic() + timeout
        last_print = 0.0

        while True:
            now = time.monotonic()
            with self._lock:
                self._raise_if_link_dead()
                reason = self._ack_error(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
                if reason:
                    raise DroneError(f"起飞命令被拒绝：{reason}")
                alt = self._state.alt_rel
                armed = self._state.armed

            # 解锁后不起飞，飞控约 10 秒会自动上锁 —— 这里显式报错，别默默等超时
            if not armed:
                raise DroneError("起飞过程中电机被上锁了（可能因为起飞命令未被接受）")

            if alt is not None and now - last_print >= 1.0:
                last_print = now
                print(f"  爬升中 {alt:5.1f} / {altitude_m:.1f} m", flush=True)

            if alt is not None and alt >= altitude_m - tolerance:
                print(f"  到达高度 {alt:.2f} m（目标 {altitude_m:.1f} m）", flush=True)
                return

            if now >= deadline:
                raise DroneError(f"超时 {timeout:.0f}s：未到达 {altitude_m:.1f} m（当前 {alt}）")

            time.sleep(self.POLL_S)  # 轮询间隔

    def goto(self, lat: float, lon: float, alt_m: float) -> None:
        """GUIDED 模式下的位置指令（SET_POSITION_TARGET_GLOBAL_INT）。

        这是"设定目标"，飞控不回应执 —— 是否到达要看遥测。
        """
        with self._send_lock:
            self.conn.mav.set_position_target_global_int_send(
                0,  # time_boot_ms，飞控会忽略
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                TYPE_MASK_POSITION_ONLY,
                int(round(lat * 1e7)),
                int(round(lon * 1e7)),
                float(alt_m),
                0, 0, 0,   # 速度（被 type_mask 忽略）
                0, 0, 0,   # 加速度（被忽略）
                0, 0,      # 偏航 / 偏航角速度（被忽略）
            )

    def fly_to(
        self,
        lat: float,
        lon: float,
        alt_m: float,
        tolerance: float = 2.0,
        timeout: float = 120.0,
        label: str = "目标点",
    ) -> None:
        """飞向一个点并等它到达（水平距离 ≤ tolerance），期间 1 Hz 打印进度。"""
        deadline = time.monotonic() + timeout
        last_send = 0.0
        last_print = 0.0

        while True:
            now = time.monotonic()

            # 1 Hz 重发目标：GUIDED 的目标会保持，但周期性重发更抗丢包
            if now - last_send >= self.GOTO_RESEND_S:
                self.goto(lat, lon, alt_m)
                last_send = now

            st = self.state
            if st.lat is not None:
                dist = distance_m(st.lat, st.lon, lat, lon)
                if now - last_print >= 1.0:
                    last_print = now
                    print(
                        f"  {label}: 距目标 {dist:6.1f} m, 高度 {st.alt_rel:.1f} m",
                        flush=True,
                    )
                if dist <= tolerance:
                    print(f"  到达 {label}（{dist:.2f} m ≤ {tolerance:.1f} m）", flush=True)
                    return

            if now >= deadline:
                raise DroneError(f"超时 {timeout:.0f}s：{label} 没飞到（还差 {dist:.1f} m）")

            time.sleep(self.POLL_S)  # 轮询间隔

    def rtl(self, timeout: float = 10.0) -> None:
        """返航（RTL 模式）。"""
        self.set_mode("RTL", timeout=timeout)

    def wait_landed(self, timeout: float = 240.0) -> None:
        """等到落地并自动上锁。"""
        def progress() -> None:
            st = self._state
            print(f"  返航中：模式={st.mode} 高度={st.alt_rel} m armed={st.armed}", flush=True)

        self._wait(
            lambda: self._state.alt_rel is not None
                    and self._state.alt_rel < 0.5
                    and not self._state.armed,
            timeout,
            "落地并上锁",
            progress=progress,
            progress_interval=5.0,
        )
        print("  已落地并上锁", flush=True)

    def param_set(self, name: str, value: float, timeout: float = 10.0) -> float:
        """设置飞控参数，并读到 PARAM_VALUE 回读值才算成功。"""
        with self._lock:
            self._params.pop(name, None)

        with self._send_lock:
            self.conn.mav.param_set_send(
                self.target_system,
                self.target_component,
                name.encode("utf8"),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )

        # 允许 0.05 的误差：REAL32 的精度本来就不够精确
        self._wait(
            lambda: name in self._params and abs(self._params[name] - value) < 0.05,
            timeout,
            f"确认参数 {name}={value}",
        )
        return self._params[name]


def _to_str(value) -> str:
    """PARAM_VALUE.param_id / STATUSTEXT.text 在不同版本里可能是 bytes。"""
    if isinstance(value, bytes):
        return value.decode("utf8", errors="replace")
    return str(value)


def require_position(drone: Drone) -> tuple[float, float]:
    """取当前水平位置；还没有位置估计就报错。"""
    st = drone.state
    if st.lat is None or st.lon is None:
        raise DroneError("还没有位置估计，无法计算航线")
    return st.lat, st.lon
