# Eight-thruster allocation

Reference: `AUV-IITK/ATAL_controller` `config/thruster_layout.yaml`, `motor_mixer_node.py`, and `firmware/esp32_thrusters/thruster_esp_final.ino`. Upstream: this checkout's `libraries/AP_Motors/AP_Motors6DOF.cpp` `SUB_FRAME_VECTORED_6DOF` case. Both use factor order [roll,pitch,yaw,vertical,forward,lateral].

`integration/vehicle_defaults.json` stores eight logical factors, motor directions, placeholder FRD positions and MCU GPIO assignments. `python3 integration/check_layout.py` reads upstream factors and verifies that the **effective** reference matrix has rank 6 and each ArduSub motor has exactly one equal row after motor reversal. It currently resolves:

| Upstream AP motor | Logical reference motor | ESP32 ID | `MOT_n_DIRECTION` |
|---:|---:|---:|---:|
| 1 | 1 | 6 | -1 |
| 2 | 8 | 7 | -1 |
| 3 | 4 | 1 | -1 |
| 4 | 5 | 3 | -1 |
| 5 | 2 | 8 | -1 |
| 6 | 7 | 5 | -1 |
| 7 | 3 | 2 | -1 |
| 8 | 6 | 4 | -1 |

The reference mixer reverses logical motors 4 and 5, and upstream `MOT_n_DIRECTION` reverses after combining factors. That is why every derived direction is -1. This is an *algebraic* mapping, not proof of physical spin direction. ArduSub `FRAME_CONFIG=2` selects Vectored-6DOF. The generic Linux RCOutput is empty, so no actuation occurs. The opt-in RCOutput observer and guarded serial bridge apply this mapping in a pseudo-terminal test; physical motor order and polarity remain unverified.

`integration/atal_motor_defaults.parm` records the provisional `FRAME_CONFIG`, `MOT_n_DIRECTION` and ROS 1300–1700 µs PWM limits. `integration/check_layout.py` verifies direction and PWM entries against the current JSON geometry/defaults and pinned upstream factor rows. Edit the JSON and parameter template together when measured geometry or polarity changes; the ESP32 ID permutation above is applied by the opt-in bridge, not by this parameter file.

The 8x6 factor matrix spans six controlled axes in exact algebra. Its nominal `position_m_frd` values do not prove real force/moment arms, thrust magnitudes, controllability under saturation or frame alignment. Actual mounting geometry, ESC wiring and polarity need human verification. No motor test has been performed or authorized.

The row mapping does **not** make the two allocators identical at saturation. The ROS 2 mixer reserves 35% headroom for heave when roll/pitch plus heave exceeds full output; `AP_Motors6DOF::output_armed_stabilizing()` instead divides every vertical combined effort by the largest absolute effort. For a synthetic roll effort of `+1` and vertical effort of `+1`, with all other efforts zero and no current limit, the mapped logical motors 2 and 3 receive normalized thrust `-0.3` from the ROS mixer but `0.0` from ArduSub; logical motors 6 and 7 receive `+1` from both. This follows from the current source implementations and default ROS headroom setting, not a hardware test. The ROS mixer also applies reverse-efficiency, expo and deadband shaping before its 1300–1700 µs PWM output, whereas this ArduSub `AP_Motors6DOF` path maps thrust linearly to its configured PWM range. Match or deliberately accept these behaviors in software simulation before connecting any output driver.

`python3 integration/simulate_allocation.py` now runs a source-derived software comparison using the editable factor table and upstream Vectored-6DOF rows. Twelve single-axis half-scale cases and one mixed unsaturated case agree exactly after motor mapping. Roll plus heave and yaw plus surge at full scale each differ by up to `0.3` normalized thrust. At `±0.5` thrust, the provisional ROS PWM curve produces `1344.1`/`1638.5` µs while the ArduSub linear curve produces `1400`/`1600` µs. The script assumes all motors enabled, unit weights, no current limit, full spool, and ROS default headroom, reverse scale, expo and deadband; it models source logic rather than executing either controller.

**Provisional software policy:** retain upstream ArduSub saturation and PWM behavior instead of copying the ROS mixer into ArduPilot. This keeps ArduSub as the authoritative allocator and avoids a vehicle-specific fork before the actual thruster curves and dynamics are measured. The 0.3 saturated difference is therefore an expected behavior change, not an equivalence claim. Before physical output, test axis authority and controller response under saturation in non-actuating simulation, then qualify thrust direction, deadband, gain and current limits with measured hardware. Revise this policy if those tests show unacceptable control loss.
