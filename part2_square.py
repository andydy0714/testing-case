#!/usr/bin/env python3
"""Part 2 — 起飞后飞一个 80 m 边长的正方形，最后 RTL 落地。"""

from __future__ import annotations

import sys

from starter.drone import Drone, DroneError, require_position, square_corners

SIDE_M = 80.0
ALT_M = 15.0
CORNER_TOL_M = 2.0
CORNER_TIMEOUT_S = 120.0


def main() -> int:
    drone = None
    try:
        print("连接 SITL ...", flush=True)
        drone = Drone()

        print("[1/3] 等待飞控就绪（EKF/GPS 收敛）", flush=True)
        drone.wait_ready(timeout=120.0)

        print("[2/3] 切 GUIDED、解锁、起飞", flush=True)
        drone.set_mode("GUIDED")
        drone.arm()
        drone.takeoff(ALT_M, timeout=60.0)

        # 起飞点就是正方形的一个角点
        lat0, lon0 = require_position(drone)
        corners = square_corners(lat0, lon0, SIDE_M)
        print(
            f"[3/3] 开始正方形：边长 {SIDE_M:.0f} m，起点 {lat0:.7f}, {lon0:.7f}",
            flush=True,
        )

        for index, (lat, lon) in enumerate(corners, start=1):
            print(f"→ 飞向角点 {index}/4：{lat:.7f}, {lon:.7f}", flush=True)
            drone.fly_to(
                lat, lon, ALT_M,
                tolerance=CORNER_TOL_M,
                timeout=CORNER_TIMEOUT_S,
                label=f"角点{index}",
            )

        print("正方形完成，执行 RTL ...", flush=True)
        drone.rtl()
        drone.wait_landed(timeout=240.0)

        print("任务完成：已落地并上锁", flush=True)
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
