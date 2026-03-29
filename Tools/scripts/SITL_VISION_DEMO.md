# SITL Vision Bridge — Minimal Working Demo

## Prerequisites

```bash
pip install pymavlink
# ArduPilot repo cloned with submodules
```

---

## Part 1 — VISION_POSITION_ESTIMATE affecting EKF position

### 1. Start SITL (terminal 1)

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --no-mavproxy
```

SITL listens on `tcp:127.0.0.1:5760`. Do **not** pass `--mavproxy` — the bridge needs that port.

---

### 2. Set parameters via MAVProxy (terminal 2)

Connect MAVProxy on the second output port SITL opens automatically:

```bash
mavproxy.py --master tcp:127.0.0.1:5762
```

Then in the MAVProxy console:

```bash
param set VISO_TYPE 1
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_VELXY 0
param set EK3_SRC1_POSZ 1
param set EK3_SRC1_YAW 1
param set GPS_TYPE 0
param set ARMING_CHECK 0
write_params
```

`GPS_TYPE 0` disables GPS so EKF is forced to rely on vision.  
`ARMING_CHECK 0` lets you arm without a full GPS lock during testing.

Reboot SITL after parameter changes:

```bash
reboot
```

---

### 3. Start the vision bridge (terminal 3)

```bash
python3 Tools/scripts/sitl_mavlink_vision_bridge.py \
    --mode vision \
    --connection tcp:127.0.0.1:5760 \
    --rate 20
```

Expected output:

```text
Waiting for heartbeat...
Heartbeat from system 1 component 1
Sending vision at 20.0 Hz on tcp:127.0.0.1:5760 (Ctrl+C to stop)
```

---

### 4. Arm and fly (MAVProxy terminal)

```bash
mode guided
arm throttle
takeoff 5
```

---

### 5. Verify EKF is consuming vision data

In MAVProxy:

```bash
graph EKF5.normInnov5
```

`normInnov5` is the EKF3 vision innovation. A value near 0 means the filter is accepting the measurements. Values consistently above 1.0 mean rejection.

Also check:

```bash
status EKF_STATUS_REPORT
```

Look for `pos_horiz_abs=True` — this confirms EKF has a good absolute horizontal position from the external source.

To watch raw message flow:

```bash
mavproxy.py --master tcp:127.0.0.1:5762 --show-raw
```

Or with pymavlink directly:

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
while True:
    msg = m.recv_match(type='VISION_POSITION_DELTA', blocking=True, timeout=2)
    if msg:
        print(msg)
```

ArduPilot re-emits `VISION_POSITION_DELTA` when it processes incoming vision — seeing this message confirms consumption, not just receipt.

---

## Part 2 — LANDING_TARGET enabling precision landing

### 1. Parameters (MAVProxy)

```bash
param set PLND_ENABLED 1
param set PLND_TYPE 1
param set PLND_EST_TYPE 0
param set ARMING_CHECK 0
write_params
reboot
```

---

### 2. Start the landing bridge (terminal 3)

```bash
python3 Tools/scripts/sitl_mavlink_vision_bridge.py \
    --mode landing \
    --connection tcp:127.0.0.1:5760 \
    --rate 20
```

---

### 3. Arm, take off, trigger precision land (MAVProxy)

```bash
mode guided
arm throttle
takeoff 10
mode land
```

With `PLND_ENABLED=1` and an active `LANDING_TARGET` stream, LAND mode activates precision landing. The vehicle will steer toward the target offset (`angle_x=0.05`, `angle_y=-0.03` in the demo) rather than descending straight down.

---

### 4. Verify precision landing is active

```bash
status LANDING_TARGET
```

You should see the message echoed back with your angle/distance values.

Watch the precision landing estimator state:

```bash
graph PRECLAND.pX PRECLAND.pY
```

Non-zero and changing values confirm the estimator is running. If both stay at 0.0 the target is not being consumed.

---

## Expected MAVLink message flow

```text
Bridge  →  VISION_POSITION_ESTIMATE (id 102)  →  ArduPilot
ArduPilot  →  VISION_POSITION_DELTA (id 11011) →  (internal, visible on output port)

Bridge  →  LANDING_TARGET (id 149)            →  ArduPilot
ArduPilot  →  EKF fuses target into PRECLAND estimator
```

Confirm message IDs are flowing with:

```python
python3 - <<'EOF'
from pymavlink import mavutil
m = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
seen = set()
while True:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg and msg.get_type() not in seen:
        seen.add(msg.get_type())
        print(msg.get_type(), msg.get_msgId())
EOF
```

---

## Failure scenario — EKF rejects vision, vehicle won't hold position

### Symptom

Vehicle drifts or falls back to barometer-only altitude. `normInnov5` stays above 1.0. No `VISION_POSITION_DELTA` on the output port.

### Cause

Timestamp mismatch. ArduPilot rejects `VISION_POSITION_ESTIMATE` if the `usec` field is too far from its own clock. The bridge uses wall-clock time which can diverge from SITL time when SITL runs faster or slower than real time.

### Debug steps

**Step 1** — check the rejection counter in MAVProxy:

```bash
param show EK3_LOG_MASK
status XKF4
```

Look for `SV` (sensor variance) or `TS` (time stamp) flags in `XKF4.SS`.

**Step 2** — log the raw timestamp delta:

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
while True:
    hb = m.recv_match(type='SYSTEM_TIME', blocking=True, timeout=2)
    if hb:
        print('AP time us:', hb.time_unix_usec, 'wall us:', int(__import__('time').time()*1e6))
```

If the delta is more than ~500 ms, ArduPilot will reject the message.

**Step 3** — fix: use AP's own time instead of wall clock in `send_vision_demo`:

```python
# replace:
usec = int((wall_t0 + t) * 1e6)
# with:
usec = int(m.time_since('SYSTEM_TIME') * 1e6)  # or fetch SYSTEM_TIME.time_unix_usec
```

**Step 4** — confirm fix by re-checking `normInnov5`. It should drop toward 0 within a few seconds of restarting the bridge.
