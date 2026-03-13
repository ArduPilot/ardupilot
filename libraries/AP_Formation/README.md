# AP_Formation Library

**FlightDock Formation Flight Controller**

GPS+UWB sensor fusion library for autonomous aerial docking research.

## Overview

This ArduPilot library implements a formation flight controller for a QuadPlane following a DHC-2 Beaver aircraft. It uses a complementary filter to blend GPS positioning with UWB ranging measurements for accurate relative positioning.

**Key Features:**
- Runs at 400Hz in ArduPilot's fast loop
- Adaptive GPS/UWB sensor fusion based on range
- Exponential smoothing for stability
- Predictive control (1.0s lookahead)
- Binary speed control (approach/maintain)

## Files

| File | Lines | Description |
|------|-------|-------------|
| `AP_Formation.h` | 109 | Class declaration, public API, parameter definitions |
| `AP_Formation.cpp` | 323 | Implementation of all methods, control algorithms |

## Architecture

### Main Control Loop (400Hz)

```cpp
void AP_Formation::update(float dt)
```

**Control Flow:**
1. Calculate GPS-derived range to lead aircraft
2. Blend GPS and UWB ranges using adaptive weighting
3. Apply exponential smoothing filter (alpha=0.3)
4. Calculate formation position with prediction (1.0s ahead)
5. Calculate velocity command toward formation position
6. Apply speed limits (binary: approach/maintain)

### Adaptive Sensor Fusion

**GPS+UWB Complementary Filter:**
```
fused_range = alpha * uwb_range + (1-alpha) * gps_range
```

**Adaptive Alpha (UWB weight):**
- `range < 50m`:    alpha = 0.8 (trust UWB 80%)
- `50m < range < 100m`: alpha = 0.8 → 0.5 (linear transition)
- `100m < range < 200m`: alpha = 0.5 → 0.3 (linear transition)
- `range > 200m`:   alpha = 0.0 (GPS only)

**Rationale:** UWB is very accurate when close (~10cm precision) but degrades with distance. GPS is consistent but less precise (~2m). Adaptive weighting optimizes for both.

### Predictive Control

**Prediction Algorithm:**
```cpp
predicted_pos = target_pos + target_vel * predict_time
```

- Default prediction time: 1.0 second
- Compensates for system lag (sensor → processing → actuation)
- Allows QuadPlane to anticipate lead aircraft maneuvers

### Formation Geometry

**Offset Calculation:**
- Trail offset: Behind lead aircraft (default 50m)
- Lateral offset: Beside lead aircraft (default 0m)
- Vertical offset: Above/below lead aircraft (default 0m)

**Coordinate Transform:**
1. Calculate lead aircraft heading from velocity
2. Transform body-frame offset to NED frame
3. Add offset to predicted lead position

## Parameters

All parameters use `FORM_` prefix for ground control station access.

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `FORM_OFFSET` | 50.0 | 10-200 | Trail distance behind leader (m) |
| `FORM_LATERAL` | 0.0 | -50 to 50 | Lateral offset (+ = right) (m) |
| `FORM_VERTICAL` | 0.0 | -50 to 50 | Vertical offset (+ = down) (m) |
| `FORM_PREDICT` | 1.0 | 0.1-3.0 | Prediction lookahead time (s) |
| `FORM_SMOOTH` | 0.3 | 0.1-1.0 | Range smoothing alpha (0=heavy, 1=none) |
| `FORM_APPROACH` | 26.0 | 10-40 | Speed when far from formation (m/s) |
| `FORM_MAINTAIN` | 24.7 | 10-40 | Speed when in formation (m/s) |
| `FORM_SPD_THRESH` | 55.0 | 10-100 | Distance for speed switch (m) |

## Public API

### Input Methods

```cpp
// Set lead aircraft position (NED frame, meters)
void set_target_position(const Vector3f &pos_ned);

// Set lead aircraft velocity (NED frame, m/s)
void set_target_velocity(const Vector3f &vel_ned);

// Set UWB range measurement (meters)
void set_uwb_range(float range_m);

// Set own position (NED frame, meters)
void set_own_position(const Vector3f &pos_ned);
```

### Control Methods

```cpp
// Main update loop - call at 400Hz with dt=0.0025
void update(float dt);

// Get commanded velocity output (m/s)
Vector3f get_velocity_command() const;

// Enable/disable formation control
void set_active(bool active);
bool is_active() const;

// Reset controller state
void reset();
```

## Usage Example

```cpp
#include <AP_Formation/AP_Formation.h>

AP_Formation formation;

void setup() {
    // Enable formation control
    formation.set_active(true);
}

void loop() {
    const float dt = 0.0025f;  // 400Hz

    // Update inputs (from sensors, inter-vehicle link, etc.)
    formation.set_own_position(current_position_ned);
    formation.set_target_position(lead_position_ned);
    formation.set_target_velocity(lead_velocity_ned);
    formation.set_uwb_range(uwb_sensor.get_range());

    // Run controller
    formation.update(dt);

    // Get velocity command
    Vector3f vel_cmd = formation.get_velocity_command();

    // Send to flight controller
    // (implementation depends on integration point)
}
```

## Integration Points

To integrate into ArduPlane:

1. **Add to build system:**
   - Add `libraries/AP_Formation` to wscript
   - Include in ArduPlane compilation

2. **Instantiate in ArduPlane:**
   ```cpp
   #include <AP_Formation/AP_Formation.h>
   AP_Formation g_formation;
   ```

3. **Call in fast loop (400Hz):**
   ```cpp
   void Plane::fast_loop() {
       // ... existing code ...
       g_formation.update(scheduler.get_loop_period_s());
   }
   ```

4. **Provide sensor inputs:**
   - GPS position from AHRS
   - Lead aircraft position from inter-vehicle link or mission
   - UWB range from rangefinder
   - Velocity from AHRS

5. **Use velocity output:**
   - Feed to guided mode velocity controller
   - Or convert to attitude/throttle commands

## Algorithm Background

Based on **FlightDock Phase 6 v1.3_stabilized** Python implementation:

- Original: DroneKit + MAVLink at 2 Hz
- This version: C++ native at 400 Hz
- Core algorithms preserved:
  * GPS+UWB complementary filter
  * Adaptive alpha calculation
  * Exponential smoothing
  * Predictive control
  * Binary speed control

**Key Improvements:**
- 200x faster update rate (2 Hz → 400 Hz)
- Lower latency (onboard vs external computer)
- Direct ArduPilot integration
- Tunable parameters via GCS

## Testing

### Unit Testing

```bash
# Compile for SITL
cd ~/FlightDock/ardupilot_formation
./waf configure --board sitl
./waf plane

# Run SITL
cd ArduPlane
../Tools/autotest/sim_vehicle.py --console --map
```

### Parameter Testing

```
# In MAVProxy
param show FORM*
param set FORM_OFFSET 30.0
param set FORM_APPROACH 28.0
```

### Flight Testing Checklist

- [ ] Verify sensor inputs updating
- [ ] Check velocity commands reasonable
- [ ] Monitor fusion alpha adaptation
- [ ] Validate smoothing reduces noise
- [ ] Test speed transitions at threshold
- [ ] Verify prediction improves tracking
- [ ] Check formation position accuracy

## Performance Targets

Based on Phase 6 Python results:

| Metric | Phase 6 (Python 2Hz) | AP_Formation (C++ 400Hz) Target |
|--------|---------------------|--------------------------------|
| Update Rate | 2 Hz | 400 Hz |
| Latency | ~1 second | <10 ms |
| Min Range | 20m | <15m |
| Oscillation | <30m amplitude | <20m amplitude |
| Tracking Error | ~5m RMS | <3m RMS |

## Known Limitations

1. **Position-based control only**
   - Outputs velocity command
   - Requires integration with guided mode
   - Not direct attitude control

2. **Simulated lead aircraft**
   - Needs real inter-vehicle communication
   - Current: expects external position source
   - Future: MAVLink stream, mission waypoint, or beacon

3. **UWB sensor integration**
   - Expects rangefinder interface
   - May need custom driver for specific UWB hardware

4. **Single follower**
   - Designed for one QuadPlane following one leader
   - Multi-aircraft formations need separate instances

## Future Enhancements

- [ ] Anti-overshoot protection (closure rate limiting)
- [ ] Confidence-based adaptation (sensor health monitoring)
- [ ] Dataflash logging integration
- [ ] MAVLink telemetry messages
- [ ] Multi-vehicle support
- [ ] Collision avoidance integration
- [ ] Geofence integration
- [ ] Failsafe handling (lost link, sensor dropout)

## References

- **FlightDock Project:** `~/FlightDock/`
- **Phase 6 Algorithm:** `controllers/phases/phase6_gps_uwb_fusion_v1_3_stabilized.py`
- **Sensor Fusion:** `sensors/fusion/gps_uwb_fusion.py`
- **Algorithm Reference:** `lua_scripts/phase7/algorithm_reference.py`

## Authors

FlightDock Research Team
Date: November 8, 2025
Version: 1.0

## License

GPLv3 (consistent with ArduPilot licensing)
