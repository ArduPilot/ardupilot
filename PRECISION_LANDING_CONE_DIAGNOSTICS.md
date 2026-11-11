# Precision Landing Cone Diagnostics Implementation

## Overview

This implementation adds comprehensive cone breach detection and diagnostics to the ArduPilot precision landing system. When the drone leaves the landing cone perimeter during descent, the transmitter receives specific status notifications.

## Feature Description

### Cone State Tracking

The precision landing system now tracks two distinct cone states:

- **CONE_INSIDE (0)**: Vehicle is within the landing cone boundaries
- **CONE_OUTSIDE (1)**: Vehicle is outside the landing cone boundaries

### Status Messages

When the drone transitions between cone states, the following notifications are sent to the transmitter (via GCS_SEND_TEXT):

#### Leaving the Cone (Transition: INSIDE → OUTSIDE)
```
PrecLand: Cone radius breached       [WARNING]
PrecLand: Descent paused              [WARNING]
PrecLand: Re-centering                [INFO]
```

#### Re-entering the Cone (Transition: OUTSIDE → INSIDE)
```
PrecLand: Recommencing descent        [INFO]
```

### Message Severity Levels

- **WARNING** (`MAV_SEVERITY_WARNING`): Indicates a significant event - cone breach or descent pause
- **INFO** (`MAV_SEVERITY_INFO`): Informational messages - re-centering or descent resume

## Implementation Details

### Files Modified

1. **libraries/AC_PrecLand/AC_PrecLand.h**
   - Added `ConeState` enum with CONE_INSIDE and CONE_OUTSIDE states
   - Added `_current_cone_state` member variable to track current cone state
   - Added `_last_cone_status_change_ms` timestamp tracking
   - Added public methods:
     - `get_cone_state()`: Query current cone state
     - `update_cone_status()`: Update cone state (called from landing mode)
   - Added private method:
     - `check_cone_status()`: Internal cone breach detection logic

2. **libraries/AC_PrecLand/AC_PrecLand.cpp**
   - Implemented `check_cone_status()`: Detects cone state transitions and sends status messages
   - Implemented `update_cone_status()`: Public wrapper for cone status updates
   - Initialized cone state in `init()`: Sets initial state to CONE_INSIDE

3. **ArduCopter/mode.cpp**
   - Added call to `copter.precland.update_cone_status()` in the landing mode control loop
   - Placement: Right after target position measurement acquisition, before descent decision

## How It Works

### Initialization
When precision landing is initialized, the cone state is set to `CONE_INSIDE` (assuming the vehicle starts in a correctly positioned state).

### Runtime Operation
During each landing update cycle:

1. Landing mode calculates:
   - `target_error_m`: Horizontal distance from target (meters)
   - `target_pos_meas_ned_m.z`: Height above target (meters)

2. Landing mode calls: `copter.precland.update_cone_status(target_error_m, height_above_target_m)`

3. Precision landing system:
   - Checks if vehicle is within cone boundaries using `is_within_landing_cone()`
   - Detects state transitions
   - Sends appropriate status messages on transitions
   - Records timestamp of state changes

### Cone Detection Logic

The cone boundary is calculated dynamically based on height:
```cpp
cone_radius_m = MAX(height_above_target_m, 0.0f) * tan(cone_angle_deg)
is_inside = horizontal_error_m <= cone_radius_m
```

## Integration Points

### Landing Mode (ArduCopter/mode.cpp)
The cone status check is integrated into the precision landing descent control:
- Called whenever target is acquired and position control is active
- Executes before the existing descent decision logic
- Does not interfere with existing control behavior

### Configuration Parameters

Existing parameters that control cone behavior:
- `PLND_CONE_ENA`: Enable/disable cone checking (0=disabled, 1=enabled)
- `PLND_CONE_ANG`: Cone angle in degrees (controls cone slope)

## Transmitter Reception

Status messages are sent via GCS_SEND_TEXT, which delivers messages to:
- Ground Control Station (GCS) telemetry connection
- MAVLink compatible transmitters/receivers
- System logs and status text display

Messages appear with:
- Timestamp (system time when sent)
- Severity level (WARNING for breaches, INFO for resumption)
- Clear, actionable text

## Example Flight Scenario

1. **Descent Begins**: Vehicle enters landing mode with target acquired
   - Cone state: CONE_INSIDE
   - No messages sent (state unchanged)

2. **Wind Gust**: Vehicle drifts outside cone radius
   - Cone state changes: CONE_INSIDE → CONE_OUTSIDE
   - Messages sent:
     ```
     PrecLand: Cone radius breached
     PrecLand: Descent paused
     PrecLand: Re-centering
     ```
   - Descent halted automatically (existing logic)

3. **Recovery**: Vehicle re-enters cone
   - Cone state changes: CONE_OUTSIDE → CONE_INSIDE
   - Message sent:
     ```
     PrecLand: Recommencing descent
     ```
   - Descent resumes automatically

## Testing Recommendations

1. **SITL Testing**: Use precision landing simulator with wind disturbances
   - Verify messages appear in system console
   - Check state transitions at various altitudes
   - Verify descent pause/resume behavior

2. **Log Analysis**: Review dataflash logs for:
   - Message timestamps
   - State transitions
   - Correlation with target position data

3. **Parameter Testing**:
   - Disable cone with `PLND_CONE_ENA=0`, verify no messages
   - Vary cone angle with `PLND_CONE_ANG`, verify boundary behavior
   - Test at multiple heights

## Backward Compatibility

This feature is fully backward compatible:
- No changes to existing parameters or APIs
- Cone diagnostics are additive (no breaking changes)
- Feature only activates when precision landing is enabled AND cone is enabled
- All existing landing behavior remains unchanged

## Future Enhancements

Potential extensions:
- Add detailed diagnostics to dataflash logs (cone breach reasons, distances)
- GCS commands to acknowledge cone breaches
- Configurable diagnostic thresholds
- Integration with event/failsafe system
