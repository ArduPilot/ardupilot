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

The reference mixer reverses logical motors 4 and 5, and upstream `MOT_n_DIRECTION` reverses after combining factors. That is why every derived direction is -1. This is an *algebraic* mapping, not proof of physical spin direction. ArduSub `FRAME_CONFIG=2` selects Vectored-6DOF. The current generic Linux RCOutput is empty, so no actuation occurs. Future RCOutput mapping must use the table and be software tested before connecting ESP32.

The 8x6 factor matrix spans six controlled axes in exact algebra. Its nominal `position_m_frd` values do not prove real force/moment arms, thrust magnitudes, controllability under saturation or frame alignment. Actual mounting geometry, ESC wiring and polarity need human verification. No motor test has been performed or authorized.
