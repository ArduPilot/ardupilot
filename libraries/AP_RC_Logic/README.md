# AP_RC_Logic — RC logic / range-function engine

A table-driven engine that activates ArduPilot auxiliary functions (`AUX_FUNC`)
— flight modes and aux-function switches — from RC channel PWM ranges and
boolean combinations of conditions, reusing the existing `RC_Channel`
aux-function action set. It brings Betaflight-style range mapping and a modes
model to ArduPilot for aux/mode switching only; it tunes no parameters and
writes no other vehicle state.

This document is the authoritative contract for the parameter schema. The
[arduconfigurator](https://github.com/j-w9/arduconfigurator) UI binds to the
parameters defined here; the firmware is the source of truth.

## Concept

Each row of the table maps a source to a target function:

- a **channel PWM range** — *when channel X is in [min,max], activate function F*
- a **link** to another function's state, or a **condition** (failsafe, armed …)

Several rows may target the same function (combined with AND/OR) or share one
channel with different ranges to select different functions or flight modes —
the Betaflight "Modes" idea, expressed as one flat table of terms that a
configurator can render rule-centric or function-centric.

## Data model (parameter schema — indices are permanent)

Registered on `AP_Vehicle` as subgroup `RCL` (index 33). One `_ENABLE` plus
`AP_RC_LOGIC_NUM_TERMS` term instances named `RCL<n>_*`.

| Param        | Type      | Meaning |
|--------------|-----------|---------|
| `RCL_ENABLE` | Int8      | 0 = engine off, 1 = on |
| `RCL<n>_FUNC`| Int16     | target `AUX_FUNC` (0 = row disabled). Reuses the `RCx_OPTION` value list. |
| `RCL<n>_OPT` | Int16     | packed flags — bits 0-1 source type (0 range, 1 link, 2 condition), bit 2 combine (0 OR, 1 AND), bit 3 negate, bits 4-5 output position (0 HIGH, 1 MIDDLE, 2 LOW) |
| `RCL<n>_SRC` | Int16     | range → RC channel 1-16; link → watched `AUX_FUNC`; condition → condition id |
| `RCL<n>_MIN` | Int16     | range: PWM low bound |
| `RCL<n>_MAX` | Int16     | range: PWM high bound |

Term index `n` is 1-based. `RCL<n>_FUNC == 0` disables the whole row.

The engine only ever activates existing `AUX_FUNC`s (modes and aux-function
switches) through `run_aux_function` — the same path a physical switch uses.
It does not write any other parameter or state.

### Multiple functions/modes on one channel

Because every term is an independent *(channel, PWM range) → function*
mapping, several terms can share one channel with different ranges to select
different functions or flight modes, exactly like a Betaflight range group.
For example, on channel 6:

```
RCL1: FUNC=RTL(4)     SRC=6  MIN=1000 MAX=1300
RCL2: FUNC=Loiter(?)  SRC=6  MIN=1400 MAX=1600
RCL3: FUNC=Fence(11)  SRC=6  MIN=1800 MAX=2100
```

### Condition source ids (`SRC` when source type = condition)

All map onto existing ArduPilot state — no Betaflight concepts:

| id | condition |
|----|-----------|
| 0  | RC failsafe |
| 1  | Battery failsafe |
| 2  | GCS failsafe |
| 3  | EKF failsafe |
| 4  | Armed |

## Evaluation semantics

Per RC frame, for each distinct `FUNC` referenced by an enabled term:

1. Evaluate each of that function's terms to a boolean:
   - **range**: `MIN <= pwm(SRC) <= MAX`
   - **link**: watched `AUX_FUNC` (`SRC`) is currently active
   - **condition**: a named vehicle condition (failsafe, armed, etc.)
   - apply `negate` if set.
2. Combine (Betaflight rule): partition into AND-terms and OR-terms.
   `active = (has AND terms AND all AND terms true) OR (any OR term true)`.
3. Debounce the combined result, then on a state edge invoke
   `run_aux_function(FUNC, active ? HIGH : LOW, Source::LOGIC)`.

### Output position / selector mode
By default a function is boolean: active rows drive it HIGH, otherwise LOW.
A row may instead request a specific **output position** via `OPT` bits 4-5
(0 HIGH, 1 MIDDLE, 2 LOW) so it can drive a **multi-position** target — for
example VTX power low/mid/high — to a chosen level rather than just on/off.

If any row for a function uses a non-default (non-HIGH) output position, that
function switches to **selector** mode: the AND/OR combine is skipped and the
**lowest-numbered active row wins**, driving its output position; if no row is
active the position is LOW. This gives condition- or range-driven multi-level
output (e.g. `RC_FAILSAFE` → LOW power, low battery → MIDDLE, normal → HIGH),
which the boolean combine cannot express. Positions map onto the target's aux
handler exactly as a physical 3-position switch would. Arm functions are
always boolean and never enter selector mode.

### Links and cycles
A link term may reference only a **range-driven** function, never another
link-driven one. This is Betaflight's own restriction and it guarantees a
single evaluation level with no cycles.

## Safety: ARM and motor-affecting functions

ARM (`ARMDISARM`, `ARMDISARM_AIRMODE`, `ARM_EMERGENCY_STOP`) and other
motor/throttle functions are supported as targets but with hard constraints:

- The engine only ever **requests** arm by calling the normal
  `run_aux_function` arm path — it runs every prearm/arm check, respects the
  safety switch and throttle position, and never sets `armed` directly or
  bypasses a check.
- **ARM is range-only**: it can never be a link target or a link source. No
  derived/chained arming. (Matches Betaflight.)
- Arm terms are honoured only with valid, fresh RC input and are debounced;
  **loss of valid RC input** while an arm term is active fails toward
  **disarm**. Reconfiguring or disabling the engine does not force a disarm
  (that would risk an in-flight motor cut and matches native `RCx_OPTION`
  behaviour); the RC-loss path above remains the arming failsafe.
- `negate` is ignored on arm terms (a negated range would be active at centre
  stick and at boot).
- When a slot is freed on reconfiguration the engine drives the function LOW
  so latching outputs (relays) do not stick on — **except** for functions
  whose LOW edge affects the motors (arm = disarm, `MOTOR_ESTOP` LOW = clear
  e-stop). Those are never toggled implicitly.
- Arming is **level/hold**, matching ArduPilot's native `ARMDISARM` aux
  function: the vehicle stays armed only while the channel is inside the arm
  range, and leaving the range disarms. Assign arm to a latching switch
  channel, **never a spring-return stick**, or the vehicle will disarm the
  moment the stick returns to centre.

## Example: reduce VTX power on RC failsafe

`VTX_POWER` (94) is now driveable through `run_aux_function`, so the engine
(and scripting/MAVLink) can set it — previously it was only readable from a
physical 6-position switch. The engine only ever emits HIGH or LOW, so it
drives VTX to maximum (HIGH) or minimum (LOW) power via
`AP_VideoTX::change_power`; the intermediate steps are only reachable from a
physical switch or scripting. The minimum step enters pit mode only while
disarmed; ArduPilot never enters pit mode while armed, so in flight the
minimum drops to the lowest configured non-zero power level (video is not
fully cut in the air).

Express "full power normally, minimum on RC failsafe" with a single
negated condition term:

```
RCL1_FUNC = 94     # VTX_POWER
RCL1_OPT  = 0x0A   # condition source (2) + negate (8)
RCL1_SRC  = 0      # condition 0 = RC failsafe
```

Not in failsafe -> negated -> HIGH -> full power. In failsafe -> LOW ->
minimum power. (Negate is allowed here; it is only forbidden on arming
terms.)

## Build / footprint

Guarded by `AP_RC_LOGIC_ENABLED` (see `AP_RC_Logic_config.h`) so it can be
compiled out on flash-constrained (e.g. F4) boards. `AP_RC_LOGIC_NUM_TERMS`
sets the table size (default 12) and trades flash/storage for capacity.

## Scope

The engine drives **mode and aux-function switching only** — it activates
existing `AUX_FUNC`s from range/link/condition logic and writes nothing else.
It deliberately does not tune parameters or touch any other vehicle state.
