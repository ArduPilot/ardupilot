# Cone Breach Diagnostics Implementation Summary

## Feature Overview

This implementation adds real-time cone breach detection and status notifications to ArduPilot's precision landing system. When the drone leaves the landing cone perimeter during descent, the transmitter is notified with specific status messages.

## Status Messages

The system sends four distinct status messages at appropriate transitions:

### When Leaving Cone (Transition: Inside → Outside)
```
1. "PrecLand: Cone radius breached"    [WARNING]
2. "PrecLand: Descent paused"          [WARNING]  
3. "PrecLand: Re-centering"            [INFO]
```

### When Re-entering Cone (Transition: Outside → Inside)
```
"PrecLand: Recommencing descent"       [INFO]
```

## Implementation Details

### Modified Files

#### 1. `libraries/AC_PrecLand/AC_PrecLand.h`

**Added Enum:**
```cpp
enum class ConeState: uint8_t {
    CONE_INSIDE = 0,      // Vehicle is inside landing cone
    CONE_OUTSIDE          // Vehicle is outside landing cone
};
```

**Added Member Variables (Private):**
- `_current_cone_state`: Current cone state (inside/outside)
- `_last_cone_status_change_ms`: Timestamp of last state transition

**Added Public Methods:**
- `get_cone_state()`: Query current cone state
- `update_cone_status(float horizontal_error_m, float height_above_target_m)`: Update cone state from landing mode

**Added Private Method:**
- `check_cone_status(...)`: Internal implementation for cone breach detection

#### 2. `libraries/AC_PrecLand/AC_PrecLand.cpp`

**Added Method: `check_cone_status()`**
```cpp
void AC_PrecLand::check_cone_status(float horizontal_error_m, float height_above_target_m)
```
- Checks if vehicle is within cone boundaries using existing `is_within_landing_cone()` logic
- Detects state transitions (inside ↔ outside)
- Sends appropriate GCS_SEND_TEXT messages on transitions
- Records timestamp of state changes

**Added Method: `update_cone_status()`**
- Public wrapper for `check_cone_status()`
- Called from landing mode

**Updated Method: `init()`**
- Initializes cone state to `CONE_INSIDE` on startup
- Sets initial timestamp to 0

#### 3. `ArduCopter/mode.cpp`

**Integration Point:** Landing mode descent control (around line 754)

**Added Call:**
```cpp
// update cone status tracking for diagnostics
copter.precland.update_cone_status(target_error_m, target_pos_meas_ned_m.z);
```

**Placement:** After target position measurement, before descent decision logic

## How It Works

### Initialization
- Cone state initialized to `CONE_INSIDE` (vehicle starts positioned correctly)
- No startup messages

### Runtime Operation
1. **Landing mode calculates:**
   - `target_error_m`: Horizontal distance to target
   - `height_above_target_m`: Z-height from target

2. **Calls:** `copter.precland.update_cone_status(...)`

3. **Precision landing system:**
   - Checks if within cone using `is_within_landing_cone()`
   - Compares to previous state
   - If transition detected:
     - Updates state variable
     - Records timestamp
     - Sends GCS_SEND_TEXT message(s)

### Cone Boundary Calculation
```
cone_radius = height * tan(cone_angle_deg)
is_inside = horizontal_error <= cone_radius
```

## Behavior

### Cone Enabled, Inside → Outside
- State changes from `CONE_INSIDE` to `CONE_OUTSIDE`
- Three messages sent (in order):
  1. Warning: "Cone radius breached"
  2. Warning: "Descent paused" 
  3. Info: "Re-centering"
- Existing control logic pauses descent (climb_rate = 0)

### Cone Enabled, Outside → Inside
- State changes from `CONE_OUTSIDE` to `CONE_INSIDE`
- One message sent:
  1. Info: "Recommencing descent"
- Existing control logic resumes descent

### Cone Disabled
- `check_cone_status()` returns immediately
- No state transitions occur
- No messages sent

## Message Delivery

Messages are sent via `GCS_SEND_TEXT()`:
- **Reception:** Ground Control Station (GCS), telemetry radios, MAVLink devices
- **Display:** System messages, logs, status text displays
- **Format:** Timestamp + Severity + Message text

## Configuration

Uses existing precision landing parameters:
- `PLND_CONE_ENA`: Enable/disable (0=disabled, 1=enabled)
- `PLND_CONE_ANG`: Cone angle in degrees

No new parameters required.

## Testing Results

### Unit Tests
- **Test File:** `libraries/AC_PrecLand/tests/test_landing_cone.cpp`
- **Tests Run:** 5 tests, all passing
- **Coverage:** 
  - Cone disabled behavior
  - Default angle calculations
  - Configurable angles
  - Boundary edge cases
  - Angle constraints

### Build Verification
- **Build Target:** ArduCopter (sitl)
- **Status:** ✓ Successful compilation
- **Size Change:** Minimal (< 1KB)

## Key Design Decisions

1. **State-Based Detection:** Uses state transitions rather than continuous checks to avoid message spam

2. **Integration Point:** Placed in landing mode where target calculations already occur - no duplicate calculations

3. **Message Sequence:** Three messages on breach (issue, action, recovery) provide clear situational awareness

4. **No API Changes:** Uses existing `GCS_SEND_TEXT()` infrastructure, no new parameters

5. **Backward Compatible:** Feature is additive; no changes to existing behavior when disabled

## Example Flight Sequence

```
1. Vehicle in land mode, target acquired
   - Cone state: INSIDE
   - Messages: None

2. Wind gust pushes vehicle outside cone
   - Cone state: INSIDE → OUTSIDE
   - Messages:
     * "Cone radius breached" (WARNING)
     * "Descent paused" (WARNING)
     * "Re-centering" (INFO)
   - Descent halts

3. Vehicle recovers position
   - Cone state: OUTSIDE → INSIDE
   - Messages:
     * "Recommencing descent" (INFO)
   - Descent resumes

4. Landing completes
   - Messages: None (state doesn't change)
```

## Verification Checklist

- [x] Code compiles without errors
- [x] All existing tests pass
- [x] New functionality integrated into landing mode
- [x] Message format is clear and actionable
- [x] State transitions handled correctly
- [x] Backward compatible (no breaking changes)
- [x] No new parameters required
- [x] Minimal performance impact

## Future Enhancement Possibilities

1. Add dataflash logging of cone breaches with details
2. GCS commands to acknowledge breaches
3. Configurable message text via parameters
4. Integration with event/failsafe system
5. Detailed diagnostics (distance outside cone, etc.)

## Related Files

- [PRECISION_LANDING_CONE_DIAGNOSTICS.md](PRECISION_LANDING_CONE_DIAGNOSTICS.md) - Detailed technical documentation
