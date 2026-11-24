"""Bounded ROS 2 GateGuidance -> ArduSub MAVLink command adapter.

Converts valid, fresh ROS GateGuidance proposals into bounded ArduSub
GUIDED velocity commands ONLY when:
1. Explicitly enabled by the operator/bridge (--enable-guidance-commands or enable()),
2. ArduSub link, attitude, and gate_link_ok are fresh,
3. Vehicle is armed and in GUIDED mode,
4. Proposal is fresh (< timeout_s), valid, and axes lie within declared limits.

On stale proposal, link loss, or APPROACH_STOP, immediately zeroes velocity.
"""

import math
import time


class GateGuidanceBridge:
    """Stateless-safe gate proposal to ArduSub GUIDED velocity converter."""

    def __init__(self, commander, max_forward_mps=0.35, max_lateral_mps=0.25,
                 max_vertical_mps=0.20, max_yaw_dps=25.0, proposal_timeout_s=0.45):
        self._cmd = commander
        self.max_forward_mps = float(max_forward_mps)
        self.max_lateral_mps = float(max_lateral_mps)
        self.max_vertical_mps = float(max_vertical_mps)
        self.max_yaw_dps = float(max_yaw_dps)
        self.proposal_timeout_s = float(proposal_timeout_s)
        self._enabled = False
        self._zero_sent = False

    @property
    def enabled(self):
        return self._enabled

    def set_enabled(self, enabled):
        self._enabled = bool(enabled)
        if not self._enabled:
            self.send_zero()

    def send_zero(self):
        if not self._zero_sent:
            try:
                self._cmd.send_guided_velocity_ned(0.0, 0.0, 0.0, yaw_rate_dps=0.0)
            except Exception:
                pass
            self._zero_sent = True

    def step(self, snapshot, now_s=None):
        """Evaluate current DashboardTelemetry snapshot and forward bounded velocity if allowed."""
        if not self._enabled:
            return {"commanded": False, "reason": "guidance bridge disabled"}
        if not snapshot.get("link_alive") or not snapshot.get("imu_alive"):
            self.send_zero()
            return {"commanded": False, "reason": "vehicle link or IMU stale"}
        if not (snapshot.get("armed_known") and snapshot.get("armed") is True):
            self.send_zero()
            return {"commanded": False, "reason": "vehicle disarmed"}
        if snapshot.get("vehicle_mode") != "GUIDED":
            self.send_zero()
            return {"commanded": False, "reason": "vehicle not in GUIDED mode"}
        if not (snapshot.get("gate_link_alive") and snapshot.get("gate_link_ok")
                and snapshot.get("gate_guidance_alive") and snapshot.get("gate_proposal_valid")):
            self.send_zero()
            return {"commanded": False, "reason": "no valid fresh gate proposal"}
        if snapshot.get("gate_state") == "APPROACH_STOP":
            self.send_zero()
            return {"commanded": False, "reason": "gate reached stop threshold"}

        fwd = snapshot.get("gate_forward")
        lat = snapshot.get("gate_lateral")
        vert = snapshot.get("gate_vertical")
        yaw = snapshot.get("gate_yaw")
        yaw_deg = snapshot.get("yaw")
        if any(v is None or not math.isfinite(float(v)) for v in (fwd, lat, vert, yaw, yaw_deg)):
            self.send_zero()
            return {"commanded": False, "reason": "non-finite proposal or yaw"}

        # Body forward/lateral -> NED velocity using current vehicle yaw
        psi = math.radians(float(yaw_deg))
        vf = max(-1.0, min(1.0, float(fwd))) * self.max_forward_mps
        vl = max(-1.0, min(1.0, float(lat))) * self.max_lateral_mps
        vd = -max(-1.0, min(1.0, float(vert))) * self.max_vertical_mps
        yr = max(-1.0, min(1.0, float(yaw))) * self.max_yaw_dps

        vn = math.cos(psi) * vf - math.sin(psi) * vl
        ve = math.sin(psi) * vf + math.cos(psi) * vl
        self._cmd.send_guided_velocity_ned(vn, ve, vd, yaw_rate_dps=yr)
        self._zero_sent = False
        return {"commanded": True, "vn": vn, "ve": ve, "vd": vd, "yaw_rate_dps": yr}
