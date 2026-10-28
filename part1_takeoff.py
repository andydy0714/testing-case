#!/usr/bin/env python3
"""Part 1 — 解锁并起飞到 15 m。"""

from __future__ import annotations

import sys

from starter.drone import Drone, DroneError

TARGET_ALT_M = 15.0
ALT_TOLERANCE_M = 0.5


def main() -> int:
    drone = None
    try:
        print("连接 SITL ...", flush=True)
        drone = Drone()

        print("[1/4] 等待飞控就绪（EKF/GPS 收敛）", flush=True)
        drone.wait_ready(timeout=120.0)

        print("[2/4] 切换到 GUIDED", flush=True)
        drone.set_mode("GUIDED")

        print("[3/4] 解锁电机", flush=True)
        drone.arm()

        print(f"[4/4] 起飞到 {TARGET_ALT_M:.0f} m", flush=True)
        drone.takeoff(TARGET_ALT_M, timeout=60.0, tolerance=ALT_TOLERANCE_M)

        print("REACHED", flush=True)
        return 0

    except DroneError as exc:
        print(f"错误：{exc}", file=sys.stderr, flush=True)
        return 1
    except KeyboardInterrupt:
        print("已手动中断", file=sys.stderr, flush=True)
        return 130
    finally:
        if drone is not None:
            drone.close()


if __name__ == "__main__":
    sys.exit(main())
