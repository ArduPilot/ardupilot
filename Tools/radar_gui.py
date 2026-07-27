#!/usr/bin/env python3
"""Live radar-sweep GUI: visualizes where ArduPilot's proximity system is
detecting obstacles, by angle and distance around the vehicle.

Subscribes to the standard OBSTACLE_DISTANCE MAVLink message (72 sectors,
one value every `increment` degrees, index 0 = dead-ahead in MAV_FRAME_BODY_FRD)
and renders it as a polar sweep with the vehicle at the centre - classic
radar display: 0 degrees (forward) at the top, angle increasing clockwise.

This reads whatever OBSTACLE_DISTANCE ArduPilot is actually sending, built
from all configured AP_Proximity instances combined (PRX1-4 for the
HLK-LD2451 corner radars) - it shows *where* something was detected, not
*which corner* saw it (OBSTACLE_DISTANCE doesn't carry per-sensor identity,
speed, or SNR - if that level of detail is ever needed, it needs a custom
MAVLink message instead of this one).

Usage:
    python3 Tools/radar_gui.py
    python3 Tools/radar_gui.py --host 127.0.0.1 --port 5762
"""
import argparse
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from pymavlink import mavutil

UNKNOWN = 65535  # UINT16_MAX sentinel for "unknown/not used"
# Hardcoded rather than mavutil.mavlink.MAVLINK_MSG_ID_OBSTACLE_DISTANCE: that
# attribute only exists on the MAVLink2 dialect module, and mavutil.mavlink
# can resolve to the MAVLink1 one (no ID >= 256) depending on connection state.
MAVLINK_MSG_ID_OBSTACLE_DISTANCE = 330
CANDIDATE_PORTS = (5762, 5763, 5760)  # tried in order; matches tools/mav_ctl.py's connect()

# --- palette -----------------------------------------------------------
BG = "#0b0f14"
PANEL = "#11161d"
GRID = "#22303c"
FG = "#d8e3ea"
MUTED = "#5f7482"
ACCENT = "#37e6c4"      # vehicle marker / sweep line
OK = "#37e6c4"
WARN = "#ffcf5c"
DANGER = "#ff5c6c"


def connect(host, port=None):
    """Connect and confirm a REAL heartbeat (target_system != 0) - some ports
    accept the TCP connection but never deliver a genuine vehicle heartbeat
    (e.g. when multiple GCS clients already share the primary MAVLink port),
    which silently breaks message-interval requests downstream. If no port is
    given, try the same candidates tools/mav_ctl.py uses."""
    ports = [port] if port else list(CANDIDATE_PORTS)
    for p in ports:
        print(f"connecting to {host}:{p} ...")
        try:
            m = mavutil.mavlink_connection(f"tcp:{host}:{p}", source_system=250, source_component=1)
            m.wait_heartbeat(timeout=6)
        except Exception as exc:  # noqa: BLE001
            print(f"  [--] {p} failed: {exc}")
            continue
        if m.target_system == 0:
            print(f"  [--] {p} gave a phantom heartbeat (sys=0) - not a real vehicle link")
            continue
        print(f"connected: tcp:{p}  sys={m.target_system} comp={m.target_component}")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                MAVLINK_MSG_ID_OBSTACLE_DISTANCE, int(1e6 / 5), 0, 0, 0, 0, 0)
        m.mav.request_data_stream_send(m.target_system, m.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)
        return m
    sys.exit("no working MAVLink port found")


def decode(msg):
    """OBSTACLE_DISTANCE -> list of (angle_deg, distance_m) for real detections
    only (skips "clear" and "unknown" sector values)."""
    increment = msg.increment_f if msg.increment_f else float(msg.increment)
    points = []
    for i, d_cm in enumerate(msg.distances):
        if d_cm == UNKNOWN or d_cm > msg.max_distance:
            continue
        points.append((msg.angle_offset + i * increment, d_cm / 100.0))
    return points


def danger_color(dist_m, margin_m):
    """Red inside the avoidance margin, fading to amber then teal further out."""
    if dist_m <= margin_m:
        return DANGER
    if dist_m <= margin_m * 2.5:
        return WARN
    return OK


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=None,
                        help=f"SITL MAVLink TCP port (default: try {list(CANDIDATE_PORTS)} in order)")
    parser.add_argument("--margin", type=float, default=2.5, help="AVOID_MARGIN in metres, for the danger-zone ring (default: 2.5)")
    args = parser.parse_args()

    m = connect(args.host, args.port)

    plt.rcParams.update({
        "figure.facecolor": BG,
        "axes.facecolor": PANEL,
        "text.color": FG,
        "axes.edgecolor": GRID,
        "font.family": "monospace",
        "font.size": 10,
    })

    fig = plt.figure("ArduPilot Radar", figsize=(11, 7.5))
    fig.suptitle("PROXIMITY RADAR", color=FG, fontsize=15, fontweight="bold", x=0.03, ha="left")
    gs = GridSpec(1, 4, figure=fig, left=0.04, right=0.97, top=0.90, bottom=0.06, wspace=0.35)

    # ------------------------------------------------------------- radar
    ax = fig.add_subplot(gs[0, 0:3], projection="polar")
    ax.set_facecolor(PANEL)
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.grid(color=GRID, linewidth=0.8, alpha=0.9)
    ax.spines["polar"].set_color(GRID)
    ax.tick_params(colors=MUTED)
    ax.set_thetagrids(range(0, 360, 45), color=MUTED)

    danger_ring, = ax.plot(np.linspace(0, 2 * np.pi, 100), [args.margin] * 100,
                           color=DANGER, linewidth=1.0, linestyle="--", alpha=0.6)
    sweep_line, = ax.plot([0, 0], [0, 1], color=ACCENT, linewidth=1.2, alpha=0.35, zorder=3)
    scatter = ax.scatter([], [], s=[], c=[], zorder=4, edgecolors="none")
    ax.plot([0], [0], marker="^", markersize=16, color=ACCENT, zorder=5,
            markeredgecolor=BG, markeredgewidth=1.2)

    # -------------------------------------------------------------- HUD
    hud = fig.add_subplot(gs[0, 3])
    hud.set_facecolor(PANEL)
    hud.set_xticks([])
    hud.set_yticks([])
    for spine in hud.spines.values():
        spine.set_color(GRID)
    hud.set_title("STATUS", loc="left", color=MUTED, fontsize=10, fontweight="bold")

    conn_text = hud.text(0.06, 0.93, "", transform=hud.transAxes, va="top", fontsize=10, color=OK)
    hits_text = hud.text(0.06, 0.83, "", transform=hud.transAxes, va="top", fontsize=10)
    nearest_text = hud.text(0.06, 0.70, "", transform=hud.transAxes, va="top", fontsize=10)
    range_text = hud.text(0.06, 0.55, "", transform=hud.transAxes, va="top", fontsize=9, color=MUTED)
    legend_y0 = 0.40
    hud.text(0.06, legend_y0, "LEGEND", transform=hud.transAxes, va="top", fontsize=9,
             color=MUTED, fontweight="bold")
    for i, (color, label) in enumerate([(DANGER, f"< {args.margin:.1f} m (margin)"),
                                        (WARN, "near"), (OK, "clear range")]):
        y = legend_y0 - 0.06 - i * 0.05
        hud.scatter([0.09], [y], s=40, c=color, transform=hud.transAxes, clip_on=False)
        hud.text(0.16, y, label, transform=hud.transAxes, va="center", fontsize=8.5, color=MUTED)
    clock_text = hud.text(0.06, 0.03, "", transform=hud.transAxes, va="bottom", fontsize=8, color=MUTED)

    state = {"last_rx": 0.0, "range": args.margin * 4}

    def update(frame):
        msg = m.recv_match(type="OBSTACLE_DISTANCE", blocking=False)
        sweep_angle = np.radians((frame * 6) % 360)
        sweep_line.set_data([sweep_angle, sweep_angle], [0, state["range"]])

        if msg is not None:
            state["last_rx"] = time.time()
            max_m = msg.max_distance / 100.0
            if max_m > 0:
                state["range"] = max_m
                if ax.get_rmax() != max_m:
                    ax.set_rmax(max_m)

            points = decode(msg)
            if points:
                angles = np.radians([p[0] for p in points])
                dists = np.array([p[1] for p in points])
                colors = [danger_color(d, args.margin) for d in dists]
                sizes = np.clip(120 - dists * 4, 25, 120)
                scatter.set_offsets(np.column_stack([angles, dists]))
                scatter.set_sizes(sizes)
                scatter.set_color(colors)
                nearest = min(zip(dists, [p[0] for p in points]))
                nearest_text.set_text(f"NEAREST\n{nearest[0]:5.1f} m @ {nearest[1]:+.0f}°")
                nearest_text.set_color(danger_color(nearest[0], args.margin))
            else:
                scatter.set_offsets(np.empty((0, 2)))
                nearest_text.set_text("NEAREST\n-- clear --")
                nearest_text.set_color(MUTED)

            hits_text.set_text(f"DETECTIONS\n{len(points)}")
            range_text.set_text(f"range {msg.min_distance/100.0:.1f}-{max_m:.1f} m")

        stale = (time.time() - state["last_rx"]) > 2.0 if state["last_rx"] else True
        conn_text.set_text("● LIVE" if not stale else "○ NO DATA")
        conn_text.set_color(OK if not stale else DANGER)
        clock_text.set_text(time.strftime("%H:%M:%S"))

        return scatter, sweep_line, danger_ring, conn_text, hits_text, nearest_text, range_text, clock_text

    ani = FuncAnimation(fig, update, interval=150, blit=False, cache_frame_data=False)  # noqa: F841
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
