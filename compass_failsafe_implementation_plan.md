# ArduCopter Failsafe_Compass Implementation Plan

## Project Overview

This document outlines the implementation plan for the Failsafe_Compass mode in ArduCopter. This mode enables the aircraft to fly in a predetermined direction using only compass and IMU data during radio failsafe events.

## Implementation Phases

### Phase 1: POC (Proof of Concept)

**Objective**: Create a basic working implementation with minimal parameters

#### Tasks:

1. **Create the new flight mode class**
   - Create `mode_failsafe_compass.cpp` file in ArduCopter directory
   - Define `ModeFailsafeCompass` class inheriting from `Mode`
   - Implement basic mode structure with required virtual methods

2. **Add mode to flight mode system**
   - Add `FAILSAFE_COMPASS` enum to flight mode definitions
   - Register the mode in the flight mode system
   - Update mode switching logic

3. **Implement basic parameters**
   - Add `FS_COMPASS_HDG` parameter (0-359 degrees) to Parameters.h
   - Add parameter definitions and validation

4. **Implement core flight behavior**
   - Altitude control: Climb to existing `FS_ALT_MIN` parameter
   - Heading control: Turn to `FS_COMPASS_HDG` using compass data
   - Pitch control: Apply fixed 10° forward pitch
   - Use existing attitude control systems

5. **Integrate with failsafe system**
   - Add failsafe_compass as an option in radio failsafe actions
   - Implement mode entry logic during radio failsafe
   - Ensure proper mode exit on manual recovery

6. **Basic testing and validation**
   - SITL testing for basic functionality
   - Verify compass heading control
   - Test altitude climb and hold behavior

### Phase 2: MVP (Minimum Viable Product)

**Objective**: Add essential configurability and safety features

#### Tasks:

1. **Expand parameter system**
   - Add `FS_COMPASS_MODE` parameter (enable/disable)
   - Add `FS_COMPASS_PITCH` parameter (5-20°, default 10°)
   - Add `FS_COMPASS_HDG_SRC` parameter (0=fixed heading, 1=home direction)

2. **Implement configurable pitch control**
   - Replace fixed 10° pitch with configurable `FS_COMPASS_PITCH`
   - Add parameter validation and range checking

3. **Investigate and implement home direction option**
   - Research how to calculate heading to home without GPS
   - Implement home direction calculation using compass and stored home position
   - Add logic to switch between fixed heading and home direction

4. **Enhanced safety features**
   - Add parameter validation on mode entry
   - Implement proper error handling for invalid configurations
   - Add status reporting to GCS

5. **Extended testing**
   - Test with various pitch angles
   - Validate home direction functionality (if feasible)
   - Test parameter validation

### Phase 3: V2 (Enhanced Version)

**Objective**: Add dynamic control and improved safety features

#### Tasks:

1. **RC channel heading adjustment**
   - Add `FS_COMPASS_HDG_CH` parameter for RC channel selection
   - Implement pre-failsafe heading monitoring from RC channel
   - Store last known heading before failsafe

2. **Throttle modification**
   - Add `FS_COMPASS_THR_MOD` parameter (±20% from hover)
   - Implement throttle adjustment for different aircraft loads
   - Integrate with altitude control system

3. **Timeout and auto-land**
   - Add `FS_COMPASS_TIMEOUT` parameter
   - Implement flight time tracking
   - Add automatic land mode transition on timeout

4. **Battery monitoring integration**
   - Integrate with existing battery failsafe system
   - Implement emergency land on critical battery
   - Add proper priority handling between different failsafe conditions

5. **Comprehensive testing**
   - Test RC channel heading adjustment
   - Validate timeout and auto-land functionality
   - Test battery failsafe integration
   - Extended flight duration testing

## Technical Implementation Details

### File Structure
```
ArduCopter/
├── mode_failsafe_compass.cpp    # Main mode implementation
├── mode.h                       # Add mode declaration
├── Copter.h                     # Add mode instance
├── Parameters.h                 # Add parameters
├── Parameters.cpp               # Parameter definitions
└── defines.h                    # Add mode enum
```

### Key Integration Points

1. **Flight Mode System**
   - Add to mode enumeration in `defines.h`
   - Add mode instance in `Copter.h`
   - Register in mode initialization

2. **Failsafe System**
   - Modify radio failsafe handler to support compass failsafe option
   - Add priority logic relative to other failsafe modes

3. **Parameter System**
   - Group all parameters under `FS_COMPASS_*` prefix
   - Implement proper parameter validation
   - Add parameter documentation

4. **Attitude Control Integration**
   - Use existing `AC_AttitudeControl` for heading and pitch
   - Leverage existing altitude control for climb and hold
   - Integrate with compass and AHRS systems

### Testing Strategy

1. **SITL Testing**
   - Basic functionality validation
   - Parameter testing
   - Failsafe scenario simulation

2. **Hardware Testing**
   - Compass accuracy verification
   - Real-world failsafe scenarios
   - Wind resistance testing

3. **Safety Testing**
   - Manual recovery verification
   - Battery failsafe interaction
   - Timeout functionality

## Success Criteria

### POC Success Criteria
- [ ] Mode can be activated during radio failsafe
- [ ] Aircraft climbs to configured altitude
- [ ] Aircraft turns to and maintains configured heading
- [ ] Aircraft maintains forward flight with fixed pitch
- [ ] Pilot can regain control and exit mode

### MVP Success Criteria
- [ ] All parameters are configurable and validated
- [ ] Home direction option works (if feasible)
- [ ] Variable pitch angles function correctly
- [ ] Enhanced safety features are operational

### V2 Success Criteria
- [ ] RC channel heading adjustment works
- [ ] Throttle modification functions properly
- [ ] Timeout and auto-land work correctly
- [ ] Battery monitoring integration is seamless

## Risks and Mitigations

### Technical Risks
- **Compass accuracy**: Mitigate with proper calibration requirements
- **Wind drift**: Accept limitation, document in user guide
- **Altitude accuracy**: Use barometer, accept inherent limitations

### Safety Risks
- **No obstacle avoidance**: Mitigate with high altitude requirement
- **No position feedback**: Document as limitation, require pilot training

### Implementation Risks
- **AHRS integration complexity**: Start with simple implementation, iterate
- **Parameter validation**: Implement comprehensive range checking
- **Mode switching logic**: Careful integration with existing failsafe system