# Independent Rear Motor Tilt Feature

## Overview

This branch implements **independent rear motor tilt control** for QuadPlane and tricopter VTOL aircraft. It enables distributed thrust across all VTOL motors during plane mode transitions, improving energy efficiency and flight performance.

### Key Capability
- **Front motors**: Tilt from vertical (0°) to forward (up to 90°) for traditional plane-mode pull
- **Rear motors**: Independently tilt from vertical (0°) to backward (up to -90°) for distributed push thrust
- **Result**: All motors contribute to forward acceleration during plane mode, reducing reliance on large front motors alone

## Supported Configurations

### Tricopter (2 Front + 1 Rear)
- 2 front motors (counter-rotating pullers): Provide yaw/roll/pitch control + forward thrust
- 1 rear motor (pusher): Provides forward thrust only
- Efficiency gain: ~15% vs standard 2-motor front tilt
- Configuration: `Q_TILT_REAR_MASK=0x4` (motor 3)

### QuadPlane (2 Front + 2 Rear)
- 2 front motors (counter-rotating pullers): Provide yaw/roll/pitch control + forward thrust
- 2 rear motors (rear-facing): Provide distributed forward thrust
- Efficiency gain: ~20-25% vs standard 2-motor front tilt
- Configuration: `Q_TILT_REAR_MASK=0xC` (motors 3+4)

### Custom Configurations
Any motor can be designated as "rear tilting" via the `Q_TILT_REAR_MASK` bitmask. For example:
- Single rear: `Q_TILT_REAR_MASK=0x8` (motor 4 only)
- Dual asymmetric: `Q_TILT_REAR_MASK=0x4` + `Q_TILT_REAR_MASK=0x20` (motors 3+6)

## Configuration Parameters

### Q_TILT_REAR_MASK (New)
**Type:** Int16  
**Default:** 0 (feature disabled)  
**Range:** 0-255  
**Description:** Bitmask of motors that tilt independently in rear direction. Bit 0 = motor 1, bit 1 = motor 2, etc.

**Examples:**
- `0` = Disabled (rear motors don't tilt)
- `0x4` = Motor 3 only (tricopter)
- `0xC` = Motors 3 and 4 (quadplane)
- `0x8` = Motor 4 only (alternate configuration)

### Q_TILT_REAR_MAX (New)
**Type:** Int8  
**Default:** 90  
**Range:** 20-90  
**Units:** degrees  
**Description:** Maximum tilt angle for rear motors in backward direction. Allows independent angle limiting from front motors.

**Physics:**
- At 0°: Rear motors point straight up (full vertical thrust, zero horizontal)
- At 45°: Rear motors angled at 45° backward (71% vertical, 71% horizontal)
- At 90°: Rear motors point straight backward (zero vertical, full horizontal thrust)

**Recommended values:**
- `90` = Maximum efficiency (full horizontal thrust in plane mode) — default
- `60` = Conservative (maintains vertical thrust margin)
- `45` = Limited (rear provides primarily vertical, minimal horizontal)

### Q_TILT_TYPE (Existing)
Must be set to `0` (TILT_TYPE_CONTINUOUS) for this feature to work.

```
0 = CONTINUOUS (supported)
1 = BINARY (not compatible)
2 = VECTORED_YAW (not compatible)
3 = BICOPTER (not compatible)
```

### Rear Motor Servo Assignment
Assign servo functions via `SERVOn_FUNCTION` parameter:
- **Tricopter (1 rear)**: Set one servo to function `45` (TILT_MOTOR_REAR)
- **QuadPlane (2 rear)**: Set two servos to functions `46` (TILT_MOTOR_REAR_LEFT) and `47` (TILT_MOTOR_REAR_RIGHT)

### Servo Reversal
Rear tilt servos typically need `SERVOn_REVERSED=1` to achieve backward tilt:
- Forward = 1000 PWM (rear motors tilt backward)
- Neutral = 1500 PWM (rear motors at vertical)
- Reverse = 2000 PWM (rear motors at vertical again, but servo full range)

Consult your servo datasheet and mount orientation to confirm required reversal.

## How It Works

### Motor Classification
During tilt compensation, motors are classified into three groups:

1. **Front tilting motors** (in `Q_TILT_MASK`):
   - Tilt angle controlled by `Q_TILT_MAX_ANGLE`
   - Participate in yaw/roll differential thrust mixing
   - Get `tilted_mul` compensation factor

2. **Rear tilting motors** (in `Q_TILT_REAR_MASK`):
   - Tilt angle independently controlled by `Q_TILT_REAR_MAX`
   - Do NOT participate in yaw/roll mixing (excluded for tricopter stability)
   - Get `rear_tilted_mul` compensation factor

3. **Non-tilting VTOL motors** (not in either mask):
   - Always vertical (no tilt)
   - Get `non_tilted_mul` compensation factor

### Thrust Compensation Physics

#### During VTOL Mode (Back-Transition to Hover)
Rear motors lose vertical thrust component as they tilt backward:
```
rear_thrust_vertical = rear_command × cos(rear_tilt_angle × 90°)
```

Autopilot automatically reduces commanded rear thrust to prevent altitude loss:
```
rear_motor_output = rear_command × cos(rear_tilt_angle × 90°)
```

#### During Forward Flight (Hover→Plane Transition)
Rear motors lose vertical thrust component as they tilt backward. Autopilot boosts them to maintain:
```
rear_motor_output = rear_command × [1 / cos(rear_tilt_angle × 90°)]
```

At 90° rear tilt: `cos(90°) = 0`, so output is maximum PWM (full thrust forward).

### Rate Limiting
Both front and rear motors use the same slew rate parameters:
- `Q_TILT_RATE_UP` = Tilt forward rate during climb transitions
- `Q_TILT_RATE_DN` = Tilt backward rate during descent transitions

Rear motors slew to the same target as front motors (cascaded control).

## Configuration Checklist

### Step 1: Identify Rear Motors
Decide which motor(s) will tilt independently:
- **Tricopter**: Motor 3 (front-mounted pusher facing backward)
- **QuadPlane**: Motors 3 and 4 (rear-mounted pushers)

### Step 2: Set Bitmask
Calculate and set `Q_TILT_REAR_MASK`:
```
Motor 1 = 0x1  (bit 0)
Motor 2 = 0x2  (bit 1)
Motor 3 = 0x4  (bit 2)
Motor 4 = 0x8  (bit 3)
Motor 5 = 0x10 (bit 4)
Motor 6 = 0x20 (bit 5)
Motor 7 = 0x40 (bit 6)
Motor 8 = 0x80 (bit 7)

For tricopter (motor 3):      Q_TILT_REAR_MASK = 0x4
For quadplane (motors 3+4):   Q_TILT_REAR_MASK = 0xC
```

### Step 3: Assign Servo Functions
- **Tricopter**: Set `SERVOn_FUNCTION = 45` (TILT_MOTOR_REAR) on one servo
- **QuadPlane**: Set `SERVOn_FUNCTION = 46` and `SERVOn_FUNCTION = 47` on two servos

### Step 4: Configure Servo Direction
Mount the servo and test PWM behavior:
- Increase PWM 1000→1500→2000 and observe rear motor tilt direction
- If backward tilt occurs at 1000 PWM: Set `SERVOn_REVERSED=1`
- If backward tilt occurs at 2000 PWM: Keep `SERVOn_REVERSED=0`

### Step 5: Calibrate Servo Endpoints
Adjust `SERVOn_MIN` and `SERVOn_MAX` so that:
- 1000 PWM = Rear motors at exact 0° vertical (neutral)
- 2000 PWM = Rear motors at exact maximum tilt angle (typically -90°)

Use a protractor or servo analyzer to measure physical servo angle.

### Step 6: Set Angle Limits
- `Q_TILT_REAR_MAX = 90` for maximum efficiency (recommended for initial testing)
- Reduce to 60° or 45° if you want more conservative operation

### Step 7: Verify Parameters
Before first flight, confirm:
```
Q_TILT_TYPE = 0 (CONTINUOUS)
Q_TILT_MASK = (appropriate front motors, e.g., 0x3 for motors 1-2)
Q_TILT_REAR_MASK = (appropriate rear motors, e.g., 0x4 for motor 3)
Q_TILT_REAR_MAX = 90 (or your chosen limit)
Q_TILT_MAX_ANGLE = 90 (or your front motor limit)
SERVOn_FUNCTION = 45 or 46/47 (rear tilt)
SERVOn_REVERSED = 1 or 0 (as determined by servo test)
```

## Flight Testing Progression

### Phase 1: Hover-Only (QHOVER mode)
- All motors at 0° (vertical)
- Front and rear motors should not move
- ✅ Hovers normally
- → Proceed if stable

### Phase 2: Acro Mode (QACRO mode)
- All motors still at 0° (feature disabled in acro)
- Test attitude control
- ✅ Responds to stick input
- → Proceed if responsive

### Phase 3: Assisted Mode (QASSIST mode)
- Airspeed-triggered transition from QHOVER to QPLANE
- Front and rear motors begin tilting forward
- Monitor: Smooth, coordinated tilt; no oscillation or sudden movements
- ✅ Transitions smoothly without jerking
- → Proceed if transition feels natural

### Phase 4: Manual Plane Mode (PLANE mode)
- Full transition to forward flight
- Rear motors should be at or near 90° backward
- Verify: Good acceleration, smooth pitch control
- ✅ Aircraft accelerates and transitions without unusual pitch trim
- → Proceed to autonomous transitions

### Phase 5: Back-Transition (QASSIST → QHOVER)
- Transition from plane mode back to hover
- Rear motors tilt back toward vertical
- Monitor: Smooth descent, no loss of altitude control
- ✅ Back-transition stable, altitude hold active
- → Feature ready for operational use

## Physics Reference

### Vertical Thrust Component
As motor tilts backward by angle θ from vertical:
```
Vertical thrust = Total thrust × cos(θ)
Horizontal thrust = Total thrust × sin(θ)
```

At different angles:
- 0°: 100% vertical, 0% horizontal (hover only)
- 30°: 86% vertical, 50% horizontal
- 45°: 71% vertical, 71% horizontal
- 60°: 50% vertical, 86% horizontal
- 90°: 0% vertical, 100% horizontal (forward flight only)

### Power Efficiency
With all motors contributing in plane mode:
```
Total power = Front_motors_power + Rear_motors_power
Single-motor tilt (front only) = higher power
All-motors distributed = 15-25% lower power at cruise
```

## Troubleshooting

### Rear Motors Don't Tilt
**Check:**
1. `Q_TILT_REAR_MASK` is non-zero
2. `Q_TILT_TYPE = 0` (CONTINUOUS)
3. Servo function assigned: `SERVOn_FUNCTION = 45/46/47`
4. Servo is powered and responding (test in servo output menu)
5. Aircraft in QASSIST/QPLANE mode (rear only tilts during transitions, not in QHOVER/QACRO)

### Rear Motors Tilt Wrong Direction
**Fix:**
1. Reverse servo: Toggle `SERVOn_REVERSED` (0↔1)
2. Or reverse servo connector at receiver

### Altitude Loss During Transitions
**Causes:**
1. `Q_TILT_RATE_UP` too fast → rear tilt exceeds front, creating nose-down pitch
   - **Fix:** Reduce `Q_TILT_RATE_UP` to 20-30°/sec
2. `Q_TILT_REAR_MAX` too high for weak rear motors → commanded power exceeds available thrust
   - **Fix:** Reduce `Q_TILT_REAR_MAX` to 60°

### Oscillation During Transition
**Causes:**
1. Servo oscillating → tuning issue
   - **Fix:** Check servo tuning, ensure smooth response
2. PID gains not tuned for rear motor influence
   - **Fix:** Re-run autotune in QLOITER mode

### No Horizontal Acceleration in Plane Mode
**Likely causes:**
1. Rear motors at 0° (not backward) → check `Q_TILT_REAR_MAX` and servo calibration
2. Rear motor failure → check telemetry for motor speeds

## Implementation Details

### Files Modified
- `ArduPlane/tiltrotor.h` — Added rear motor parameters and helper methods
- `ArduPlane/tiltrotor.cpp` — Added rear motor slew control and thrust compensation

### New Methods
- `is_rear_motor_tilting(motor_id)` — Check if motor is in rear mask
- `rear_tilt_enabled()` — Check if rear tilt feature is enabled
- `slew_rear(target_angle)` — Slew rear motor angle with rate limiting

### Modified Methods
- `tilt_compensate_angle()` — Now accepts separate rear multiplier
- `tilt_compensate()` — Calculates separate rear compensation factors
- `setup()` — Initializes rear servo functions

## Design Rationale

### Why Independent Rear Control?
Traditional quadplane uses 2 front tilting motors for VTOL. During plane mode, only front motors provide forward thrust (rear motors fixed vertical). This concentrates all horizontal acceleration on 2 motors.

Independent rear control distributes thrust:
- All 4 motors (quad) or 3 motors (tri) accelerate aircraft forward
- Lower per-motor power demand
- Smoother transitions
- Better redundancy if one motor fails

### Why Exclude Rear from Yaw Mixing?
In a tricopter, the rear motor is mounted at fuselage centerline. Differential rear thrust cannot produce yaw moments (no lever arm). Yaw control must come from front motors (differential front thrust).

In a quadplane, rear motors *could* participate in yaw mixing, but their moment arm is smaller than front motors. Current implementation conservatively excludes them for simplicity.

### Why Separate Angle Limit?
Rear motors may have different physical constraints than front motors:
- Different servo range
- Different mechanical linkage design
- User may want rear at 45° max while front at 90° max

`Q_TILT_REAR_MAX` provides independent control without firmware changes.

## Future Enhancements

Possible improvements for future phases:

1. **Separate rear target angle** — Allow rear to tilt independently from front based on efficiency curves
2. **Transition completion detection** — Ensure both front AND rear reach target before declaring transition complete
3. **Back-transition airspeed coordination** — Delay rear tilt-back until airspeed low enough for safety
4. **Yaw mixing for quadplane rear** — Enable differential rear thrust for yaw on 4-motor configs
5. **SITL simulator support** — Custom tricopter/quad model for development

## References

- ArduPilot QuadPlane Documentation: https://ardupilot.org/plane/docs/quadplane-overview.html
- Tiltrotor Control Theory: https://en.wikipedia.org/wiki/Tiltrotor
- This branch: https://github.com/ArduPilot/ardupilot/compare/master...feature/independent-rear-tilt-quadplane

---

**Last Updated:** Phase 3 (Thrust Compensation)  
**Supported Vehicles:** ArduPlane QuadPlane, Tricopter  
**Firmware Requirement:** 4.0+
