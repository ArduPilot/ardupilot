#!/usr/bin/env python3
"""
Send VISION_POSITION_ESTIMATE or LANDING_TARGET to ArduPilot for SITL / lab tests.

Typical Copter setup (MAVProxy parameter console or defaults file):

  Vision (EKF uses external nav for horizontal position):
    VISO_TYPE 1
    EK3_SRC1_POSXY 6
    Optional: EK3_SRC1_POSZ / EK3_SRC1_YAW — see AP_NavEKF_Source.

  Precision landing (MAVLink target + LAND mode):
    PLND_ENABLED 1
    PLND_TYPE 1

Run SITL with a free TCP port for this script (no MAVProxy on 5760), e.g.:
  Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --no-mavproxy

Then:
  python3 Tools/scripts/sitl_mavlink_vision_bridge.py --connection tcp:127.0.0.1:5760

Or bind to default MAVProxy UDP forward (often port 14550):
  python3 Tools/scripts/sitl_mavlink_vision_bridge.py --connection udpin:127.0.0.1:14550

Replace the demo trajectories with your estimator output.

AP_FLAKE8_CLEAN
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from pymavlink import mavutil

# If not installed: pip install pymavlink


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument(
        '--connection',
        default='tcp:127.0.0.1:5760',
        help='pymavlink connection string (tcp / udpin / udpout)',
    )
    p.add_argument(
        '--mode',
        choices=('vision', 'landing'),
        default='vision',
        help='vision: VISION_POSITION_ESTIMATE; landing: LANDING_TARGET',
    )
    p.add_argument('--rate', type=float, default=20.0, help='message rate (Hz)')
    p.add_argument(
        '--source-system',
        type=int,
        default=255,
        help='MAVLink source system id',
    )
    p.add_argument(
        '--source-component',
        type=int,
        default=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,
        help='MAVLink source component id',
    )
    return p.parse_args(argv)


def wait_heartbeat(m: mavutil.mavlink_connection) -> None:
    print('Waiting for heartbeat...', flush=True)
    m.wait_heartbeat()
    print(
        f'Heartbeat from system {m.target_system} component {m.target_component}',
        flush=True,
    )


def build_pose_covariance() -> list[float]:
    """Diagonal-ish pose covariance (21 floats); NaN means unknown to ArduPilot."""
    cov = [float('nan')] * 21
    # position variance NED (m^2)
    for i in (0, 6, 11):
        cov[i] = 0.05
    # attitude variance roll, pitch, yaw (rad^2)
    for i in (15, 18, 20):
        cov[i] = 0.01
    return cov


def send_vision_demo(
    m: mavutil.mavlink_connection,
    wall_t0: float,
    cov: list[float],
) -> None:
    t = time.time() - wall_t0
    usec = int((wall_t0 + t) * 1e6)
    x = 2.0 * math.sin(0.3 * t)
    y = 2.0 * math.cos(0.3 * t)
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    m.mav.vision_position_estimate_send(
        usec,
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
        cov,
    )


def send_landing_target_demo(m: mavutil.mavlink_connection) -> None:
    """
    Body-FRD: target slightly forward/right; distance in metres.
    Requires PLND_TYPE=1 on Copter; use LAND mode when testing.
    """
    angle_x = 0.05
    angle_y = -0.03
    distance = 8.0
    m.mav.landing_target_send(
        int(time.time() * 1e6),
        1,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        angle_x,
        angle_y,
        distance,
        0.01,
        0.01,
    )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.rate <= 0:
        print('ERROR: --rate must be positive', file=sys.stderr)
        return 1

    period = 1.0 / args.rate
    m = mavutil.mavlink_connection(
        args.connection,
        source_system=args.source_system,
        source_component=args.source_component,
    )
    wait_heartbeat(m)

    cov = build_pose_covariance()
    wall_t0 = time.time()
    print(
        f'Sending {args.mode} at {args.rate} Hz on {args.connection} (Ctrl+C to stop)',
        flush=True,
    )

    try:
        while True:
            if args.mode == 'vision':
                send_vision_demo(m, wall_t0, cov)
            else:
                send_landing_target_demo(m)
            time.sleep(period)
    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
