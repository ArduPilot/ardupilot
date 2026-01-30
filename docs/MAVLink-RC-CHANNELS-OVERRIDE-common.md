# Feature: RC_CHANNELS_OVERRIDE in common MAVLink

## Summary

**RC_CHANNELS_OVERRIDE** (message ID 70) is already part of the **MAVLink common message set** (`common.xml`) in the upstream [MAVLink repository](https://github.com/mavlink/mavlink). It is documented under [Common Messages](https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE).

This note clarifies the current state and why having it in common is useful.

## Rationale (from feature request)

- **QGC**: Having the message in common makes it easier to improve and add joystick features in QGroundControl without depending on the ArduPilot dialect.
- **MANUAL_CONTROL vs RC_CHANNELS_OVERRIDE**: The common dialect has `MANUAL_CONTROL`, but it exposes fewer channels (e.g. 4 axes + AUX1–6). For robotic arms, gimbals, or full RC-style control, overriding **any** RC channel is more flexible than a fixed set of axes.
- **Use cases**: Joystick control, robotic arms, gimbals, and any application that needs full freedom to override RC channels benefit from a generic 16-channel (or 18-channel with extensions) override in common.

## Current state

### Upstream MAVLink (mavlink.io)

- **RC_CHANNELS_OVERRIDE** is defined in **common.xml** (message ID 70).
- It includes:
  - `target_system`, `target_component`
  - `chan1_raw` … `chan8_raw` (base)
  - MAVLink 2 extension fields: `chan9_raw` … `chan18_raw`
- Semantics: UINT16_MAX = “ignore”; 0 (ch 1–8) or UINT16_MAX-1 (ch 9–18) = “release back to RC”.

### ArduPilot

- MAVLink headers are generated from **modules/mavlink** (submodule: [ArduPilot/mavlink](https://github.com/ArduPilot/mavlink)), using `message_definitions/v1.0/all.xml`.
- Handling is in **libraries/GCS_MAVLink**:
  - `GCS_Common.cpp`: `handle_rc_channels_override()`, message switch `MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE`.
  - Supports all 16 channels and follows the “ignore”/“release” semantics above.

So from the **upstream** MAVLink perspective, RC_CHANNELS_OVERRIDE is already common; no “move” is required there.

## If you want to promote or clarify “common” usage

1. **QGC / other GCS**
   - They can rely on **common** only (no ArduPilot dialect) for RC_CHANNELS_OVERRIDE, since it is in common.xml.
   - Any GCS that uses upstream MAVLink common already has the message.

2. **ArduPilot mavlink fork**
   - If **ArduPilot/mavlink**’s `all.xml` (or dialect layout) ever had RC_CHANNELS_OVERRIDE only in an ArduPilot-specific file, then the change would be: ensure the message is taken from **common** (or add it to common in that fork and include common in the build). That would be done in the [ArduPilot/mavlink](https://github.com/ArduPilot/mavlink) repo, not in the main ArduPilot codebase.

3. **Upstream MAVLink**
   - No protocol change needed for “moving” RC_CHANNELS_OVERRIDE to common; it is already there.
   - Possible follow-ups: documentation or a short note on [Manual Control / RC input](https://mavlink.io/en/services/manual_control.html) that RC_CHANNELS_OVERRIDE in common is the right message for full channel override (joystick, gimbals, arms, etc.) vs MANUAL_CONTROL’s fixed axes.

## References

- [MAVLink common messages – RC_CHANNELS_OVERRIDE](https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE)
- [Manual Control (joystick)](https://mavlink.io/en/services/manual_control.html)
- ArduPilot: `libraries/GCS_MAVLink/GCS_Common.cpp` (`handle_rc_channels_override`), `GCS.h`
- ArduPilot mavlink submodule: `modules/mavlink`, wscript uses `all.xml` for code generation
