# ArduCopter Failsafe_Compass Mode Implementation

## Overview

Failsafe_Compass is a minimalistic failsafe mode for ArduCopter that enables the aircraft to fly in a predetermined direction using only compass and IMU data, without requiring GPS or other positioning systems. This mode activates during radio failsafe events to help the aircraft escape from its current location and potentially return toward the launch area.

## Core Concept

When radio communication is lost, the aircraft will:

1. Turn to a preset heading
2. Climb to a safe altitude
3. Fly forward using pitch control
4. Continue until pilot regains control

## Implementation Phases

### POC (Proof of Concept)

**Goal**: Validate basic heading-based flight without position control

**Features**:

- Single parameter: `FS_COMPASS_HDG` (0-359 degrees)
- Fixed pitch angle: 5 degrees
- Use existing RTL altitude parameter for climb target
- No adjustable parameters during flight

**Behavior**:

```
On Radio Failsafe:
1. Climb to FS_ALT_MIN (existing param)
2. Turn to FS_COMPASS_HDG
3. Apply 5° forward pitch
4. Continue until manual recovery
```

### MVP (Minimum Viable Product)

**Goal**: Add essential configurability and safety features

**New Parameters**:

- `FS_COMPASS_HDG`: Target heading (0-359°)
- `FS_COMPASS_PITCH`: Forward pitch angle (5-20°, default 5°)
- `FS_COMPASS_HDG_SRC`:
    - 0 = Use FS_COMPASS_HDG value
    - 1 = Use home direction if available (investigate feasibility)

**Features**:

- Configurable pitch angle
- Option to use home direction (if compass heading to home is available without GPS)
- Reuse existing failsafe altitude parameters

**Behavior**:

```
On Radio Failsafe (if FS_COMPASS_MODE failsafe action is selected. Same as POC):
1. Check heading source:
   - If HDG_SRC=1 and home direction available: use it
   - Otherwise: use FS_COMPASS_HDG
2. Climb to failsafe altitude
3. Turn to target heading
4. Apply FS_COMPASS_PITCH forward pitch
5. Continue until manual recovery
```

### V2 (Enhanced Version)

**Goal**: Add dynamic control and improved safety

**Additional Parameters**:

- `FS_COMPASS_HDG_CH`: RC channel for heading adjustment (0=disabled)
- `FS_COMPASS_THR_CTRL_ENABLED`: Enables or disables advanced throttle control
- `FS_COMPASS_TARGET_THR`: Target throttle
- `FS_COMPASS_PITCH_MIN`: Target throttle
- `FS_COMPASS_PITCH_MAX`: Target throttle

**Features**:

- RC-adjustable heading (when link available before failsafe).
- Throttle target for heavy/light loads. Allows to maintain throttle if possible and adjust pitch manually.

**Enhanced Behavior**:

```
Before failsafe:
- Monitor FS_COMPASS_HDG_CH for heading updates

On Radio Failsafe:
1. Use last known heading from RC channel (if configured)
2. Climb to altitude
3. Maintain heading, pitch, and throttle.
	1. We have to maintain the throttle here, but it will be simpler to adjust pitch, allow the Z controller to maintain throttle, and adjust pitch if needed.
	   Details: At the beginning, select a pitch between the min and max params. Monitor throttle if it's +- 5% from target - continue as is. If not, start increasing or decreasing pitch correspondingly. We probably need PID or other controller here.
4. Continue until manual recovery
```

## Technical Implementation Notes

### Altitude Control

- Leverage existing Z-axis controller
- Use barometer for altitude hold
- Reuse RTL or failsafe altitude parameters

### Heading Control

- Use compass data with IMU fusion
- Implement turn rate limiting for smooth transitions
- Consider magnetic declination

### Pitch Control

- Direct pitch angle command
- No position or velocity feedback required
- Let natural aircraft dynamics determine speed

### State Management

- New flight mode: `FAILSAFE_COMPASS`
- Integrate with existing failsafe system
- Clear mode exit only through manual pilot intervention

## Safety Considerations

### Limitations

- No obstacle avoidance
- No position awareness
- Wind drift not compensated
- Altitude based only on barometer

### Mitigations

- Climb to safe altitude first
- Conservative pitch angles
- Battery monitoring
- Pilot can override immediately upon RC recovery

## Integration Points

### Failsafe System

- Add new failsafe action option
- Priority relative to other failsafe modes
- Compatibility with GCS failsafe

### Flight Mode System

- Register as new flight mode
- Ensure proper mode switching logic
- Display status to GCS

### Parameter System

- Group parameters under FS_COMPASS_*
- Set reasonable defaults
- Validate parameter ranges

## Testing Plan

### POC Testing

1. Test if a drone flies well in alt hold.
2. Detect where north is and a safe pitch during that flight.
3. Hardcode the needed pitch and set the needed heading parameter. Set the mode to some switch.
4. Fly in the opposite direction. Enable failsafe mode manually.
5. The drone should go more or less back.
6. Disable failsafe mode manually.

### MVP Testing

1. Parameter validation
2. Home heading detection (if implemented)
3. Various pitch angles
4. Wind resistance testing

### V2 Testing

1. RC channel heading adjustment
2. Timeout and auto-land
3. Battery failsafe integration
4. Extended flight duration