# ArduPPM_legacy (historical reference, not built)

This directory is **not referenced by any build file and is not compiled**. ArduPPM (the
standalone AVR coprocessor firmware that decoded RC PWM into PPM for the APM1.x, PhoneDrone, and
APM2.x boards) was removed from this repository along with those boards; the servo-input decode
ISR this targets (historically `Tools/ArduPPM/Libraries/PPM_Encoder.h`, `ISR(SERVO_INT_VECTOR)`,
SERVO PWM mode) no longer exists here.

**On provenance:** `servo_isr_reference.h` and `servo_isr_ours.h` are not copies of the literal
historical AVR source. They're a from-scratch reconstruction of the decode logic (edge detection,
jitter filter, channel bookkeeping, and PR #6's throttle-low failsafe) matching how
[PR #6](https://github.com/ArduPilot/ardupilot/pull/6) described and modified it, with the
AVR-specific plumbing (the raw interrupt vector, port/timer register reads) replaced by plain
function arguments so it compiles and runs off-target. `servo_isr_ours.h` implements a 256-byte
lowest-set-bit lookup table instead of PR #6's per-bit scan, avoiding empty scan iterations when
only one or two of the eight channels changed in a given interrupt.

Verified `servo_isr_ours` against `servo_isr_base` with a differential test (not included here,
run separately): 5000 simulated interrupt sequences of 50 calls each (250,000 total calls),
covering random pin-change patterns, simultaneous multi-channel edges, and mid-sequence
failsafe-flag changes. Zero mismatches in `ppm[]`, `servo_input_connected[]`, or
`servo_input_errors` across the entire run.
