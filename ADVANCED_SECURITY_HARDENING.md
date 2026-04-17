# ArduPilot Advanced Security Hardening Framework

This document describes the comprehensive security hardening approach for ArduPilot,
including property-based testing, fuzzing, static analysis, and formal verification.

## Table of Contents

1. [Property-Based Testing with Hypothesis](#1-property-based-testing-with-hypothesis)
2. [AFL++ Fuzzing Campaign](#2-afl-fuzzing-campaign)
3. [Static Analysis Pipeline](#3-static-analysis-pipeline)
4. [Formal Verification](#4-formal-verification)
5. [Campaign Structure](#5-campaign-structure)

---

## 1. Property-Based Testing with Hypothesis

### Overview

Instead of writing one test per bug, we define **properties** that must hold across
thousands of automatically generated test cases. Hypothesis generates edge cases and
shrinks failures to minimal counterexamples.

### Installation

```bash
pip3 install hypothesis pytest
```

### Quick Start

```bash
# Run property-based tests
cd ArduPilot/Tools/autotest
python3 test_property_faults.py

# Run with more examples
python3 test_property_faults.py --hypothesis-examples=5000
```

### Fault Injection Types

#### Timing Faults
- Slow sensor update
- Delayed command loop
- Skipped scheduler tick

#### Data Corruption
- Mission file corruption
- Malformed parameter values
- Bad fence data

#### Sensor Faults
- GPS jump/freezes
- IMU bias ramp
- Barometer drift
- Magnetometer spikes

#### Link Impairment
- MAVLink delay
- Packet loss
- Bandwidth limitation

### Example Test

```python
from hypothesis import given, settings, strategies as st

@given(
    t_sec=st.integers(min_value=5, max_value=180),
    dx=st.floats(min_value=-100, max_value=100, allow_nan=False, allow_infinity=False),
    dy=st.floats(min_value=-100, max_value=100, allow_nan=False, allow_infinity=False),
    loss=st.integers(min_value=0, max_value=50),
)
@settings(max_examples=2000, deadline=60000)
def test_gps_jump_with_link_loss(t_sec, dx, dy, loss):
    """Test vehicle response to GPS jump combined with link impairment"""
    scenario = {
        "vehicle": "copter",
        "map": "windy_coast",
        "mission": "rtl_after_wp3",
        "faults": [
            {
                "type": "gps_jump",
                "t_sec": t_sec,
                "dx_m": dx,
                "dy_m": dy
            },
            {
                "type": "link_impairment",
                "start_t_sec": max(1, t_sec - 3),
                "duration_sec": 15,
                "netem": {"loss_pct": loss}
            }
        ]
    }
    
    result = run_sitl_scenario(scenario)
    
    # Property assertions
    assert result.no_in_air_disarm, "Vehicle disarmed while in air"
    assert result.mode_end in {"RTL", "LAND", "LOITER"}, f"Bad end mode: {result.mode_end}"
    assert result.max_roll_deg < 60, f"Excessive roll: {result.max_roll_deg}"
    assert result.max_pitch_deg < 60, f"Excessive pitch: {result.max_pitch_deg}"
```

### Success Properties

Properties that must hold for all generated test cases:

```python
success_properties = {
    "no_disarm_in_air": True,
    "no_geofence_breach": True,
    "mode_eventually_in": ["RTL", "LAND", "LOITER"],
    "max_attitude_error_deg_lt": 35,
    "no_oscillation": True,
    "eventual_landing": True,
}
```

### Running Campaigns

```bash
# Short campaign (1 hour)
python3 test_property_faults.py --duration=3600

# Long campaign (24 hours)
python3 test_property_faults.py --duration=86400 --output-dir=/data/fuzz-results

# Distributed campaign
python3 test_property_faults.py --workers=8 --seed=12345
```

---

## 2. AFL++ Fuzzing Campaign

### Overview

AFL++ is a coverage-guided fuzzer for finding edge-case bugs in input parsers.
Use it for:
- Mission file parsers
- Parameter import/export
- MAVLink message handling
- Log parsing
- Scripting/plugin interfaces

### Installation

```bash
# Install AFL++
sudo apt-get install afl++ llvm

# Or build from source
git clone https://github.com/AFLplusplus/AFLplusplus
cd AFLplusplus
make distrib-install
```

### Build for Fuzzing

```bash
# Compile ArduPilot with AFL++ instrumentation
export CC=afl-clang-fast
export CXX=afl-clang-fast++
export CFLAGS="-O3 -fsanitize=address,undefined"

./waf configure --board sitl
./waf copter
```

### Fuzzing Targets

#### Mission File Parser

```bash
# Create seed corpus
mkdir -p fuzz-seeds/mission
cp Tools/autotest/missions/*.waypoint fuzz-seeds/mission/

# Start fuzzing
afl-fuzz -i fuzz-seeds/mission -o fuzz-output/mission \
    ./build/sitl/bin/fuzz_mission_parser @@
```

#### Parameter Parser

```bash
afl-fuzz -i fuzz-seeds/params -o fuzz-output/params \
    ./build/sitl/bin/fuzz_param_parser @@
```

#### MAVLink Handler

```bash
afl-fuzz -i fuzz-seeds/mavlink -o fuzz-output/mavlink \
    -m none \
    ./build/sitl/bin/fuzz_mavlink_handler @@
```

### Crash Analysis

```bash
# Analyze crash
afl-showmap -i fuzz-output/mission/crashes/crash-001 \
    -o /tmp/map -- ./build/sitl/bin/fuzz_mission_parser @@

# Minimize crash corpus
afl-cmin -i fuzz-output/mission/crashes -o minimized-crashes \
    -- ./build/sitl/bin/fuzz_mission_parser @@
```

### Continuous Fuzzing

```yaml
# .github/workflows/fuzz-afl.yml
name: AFL++ Fuzzing

on:
  schedule:
    - cron: '0 3 * * *'  # Daily at 3 AM

jobs:
  fuzz:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Install AFL++
        run: |
          sudo apt-get update
          sudo apt-get install -y afl++ llvm
      - name: Build fuzzers
        run: |
          export CC=afl-clang-fast
          export CXX=afl-clang-fast++
          ./waf configure --board sitl
          ./waf fuzz_targets
      - name: Run fuzzers
        run: |
          timeout 3600 afl-fuzz -i seeds -o output ./fuzz_mission_parser @@
      - name: Upload crashes
        uses: actions/upload-artifact@v3
        with:
          name: afl-crashes
          path: output/crashes/
```

---

## 3. Static Analysis Pipeline

### Toolchain

| Tool | Purpose | CI Enforcement |
|------|---------|----------------|
| clang-tidy | C++ best practices, bug patterns | Hard fail |
| Clang Static Analyzer | Path-sensitive analysis | Hard fail |
| Cppcheck | UB, dangerous constructs | Advisory → Hard |
| Frama-C | C subset verification | Advisory |
| CodeChecker | Result management | Optional |

### Configuration

#### clang-tidy (.clang-tidy)

```yaml
---
Checks: >-
  -*,
  bugprone-*,
  cert-*,
  clang-analyzer-*,
  cppcoreguidelines-*,
  misc-*,
  performance-*,
  portability-*,
  readability-*,
  -readability-magic-numbers,
  -readability-identifier-length
WarningsAsErrors: '*'
HeaderFilterRegex: '.*'
AnalyzeTemporaryDtors: false
FormatStyle: none
CheckOptions:
  - key: cert-oop54-cpp.WarningThreshold
    value: '4'
```

#### CI Commands

```bash
# Generate compile_commands.json
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -B build

# Run clang-tidy
run-clang-tidy -p build \
    -checks='-*,clang-analyzer-*,bugprone-*,cert-*' \
    -warnings-as-errors='*' \
    -j$(nproc)

# Run Clang Static Analyzer
scan-build --status-bugs -o scan-build-report \
    cmake --build build

# Run Cppcheck
cppcheck --enable=warning,style,performance,portability \
    --std=c++17 \
    --inline-suppr \
    --xml-version=2 \
    --project=build/compile_commands.json \
    2> cppcheck-report.xml

# Run Cppcheck MISRA addon
cppcheck --dump src/AC_AttitudeControl/AC_AttitudeControl.cpp
python3 cppcheck/addons/misra.py \
    --rule-texts=cppcheck/addons/misra_rule_texts.txt \
    src/AC_AttitudeControl/AC_AttitudeControl.cpp.dump
```

### Critical Subset Policy

Apply strictest rules to safety-critical code only:

**Critical subset:**
- Attitude controller
- Rate controller
- Mode transitions
- Failsafe decision logic
- Estimator interface wrappers

**Policy:**
- All new code must be clean
- Legacy violations tracked with ratchet
- Gradual reduction of technical debt

---

## 4. Formal Verification

### Level A: TLA+ for State Machines

#### Installation

```bash
# Install TLAPS (TLA+ Proof System)
# See https://tlapl.us/ for installation instructions

# Install Apalache for symbolic checking
curl -L https://github.com/informalsystems/apalache/releases/latest/download/apalache-linux.tar.gz | tar xz
```

#### Mode Management Spec (ModeManager.tla)

```tla
------------------------ MODULE ModeManager ------------------------
EXTENDS Integers, Sequences, TLC

(* Vehicle mode states *)
VARIABLES current_mode, armed, gps_valid, terrain_valid

(* Mode set *)
Modes == {"STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "RTL", "LAND", "LOITER", "DRIFT"}

(* Safety properties *)
TypeInvariant ==
    /\ current_mode \in Modes
    /\ armed \in {TRUE, FALSE}
    /\ gps_valid \in {TRUE, FALSE}

(* Arming rules *)
ArmingAllowed ==
    ~armed =>
        /\ gps_valid
        /\ terrain_valid
        /\ current_mode \in {"STABILIZE", "LOITER"}

(* Mode transition rules *)
ModeTransitions ==
    /\ current_mode = "AUTO" =>
        \/ gps_valid
        \/ current_mode' = "RTL"
    /\ current_mode = "RTL" =>
        \/ gps_valid
        \/ current_mode' = "LAND"

(* Failsafe properties *)
FailsafePriority ==
    \/ ~gps_valid => current_mode \in {"LAND", "ALT_HOLD"}
    \/ ~terrain_valid => current_mode # "AUTO"

(* Liveness: eventually reach safe mode *)
EventualSafety ==
    []<>(~gps_valid => current_mode \in {"LAND", "RTL", "LOITER"})

====================================================================
```

#### Run Model Checker

```bash
# TLC model checker
java -jar tla2tools.jar -workers 4 ModeManager.tla

# Apalache symbolic checking
apalache-mc check --inv=EventualSafety ModeManager.tla
```

### Level B: Kind 2 for Synchronous Logic

#### Sensor Voter Spec (voter.lustre)

```lustre
node sensor_voter(
    s1_valid: bool; s1_value: real;
    s2_valid: bool; s2_value: real;
    s3_valid: bool; s3_value: real
)
returns (
    voted_value: real;
    voted_valid: bool;
    fault_detected: bool
);
var
    consensus_threshold: real;
let
    consensus_threshold = 0.5;  -- 0.5 deg/s threshold
    
    fault_detected = 
        (s1_valid and s2_valid and abs(s1_value - s2_value) > consensus_threshold) or
        (s2_valid and s3_valid and abs(s2_value - s3_value) > consensus_threshold) or
        (s1_valid and s3_valid and abs(s1_value - s3_value) > consensus_threshold);
    
    voted_valid = 
        (if s1_valid then 1 else 0) +
        (if s2_valid then 1 else 0) +
        (if s3_valid then 1 else 0) >= 2;
    
    voted_value = 
        if voted_valid then
            (s1_value + s2_value + s3_value) / 3.0
        else
            0.0;
tel

-- Contract
--%contract sensor_voter_contract =
--%   assumes s1_valid, s2_valid, s3_valid
--%   ensures voted_valid
--%   ensures abs(voted_value - s1_value) < consensus_threshold
--%   ensures abs(voted_value - s2_value) < consensus_threshold
--%   ensures abs(voted_value - s3_value) < consensus_threshold
```

#### Run Kind 2

```bash
kind2 --input lustre voter.lustre --smt_solver z3
```

### Level C: CBMC for Code Proofs

#### PID Controller Proof

```c
// libraries/AC_PID/AC_PID.cpp - proof harness

#include <assert.h>
#include <math.h>

typedef struct {
    float kp, ki, kd;
    float integ;
    float integ_min, integ_max;
    float last_error;
} pid_t;

float pid_step(pid_t *s, float err, float dt) {
    // Derivative
    float deriv = (err - s->last_error) / dt;
    
    // Integral with anti-windup
    float integ_input = err;
    s->integ += integ_input * s->ki * dt;
    
    // Clamp integral
    if (s->integ > s->integ_max) s->integ = s->integ_max;
    if (s->integ < s->integ_min) s->integ = s->integ_min;
    
    s->last_error = err;
    
    float out = s->kp * err + s->integ + s->kd * deriv;
    
    return out;
}

// CBMC proof harness
void harness() {
    pid_t s;
    __CPROVER_assume(s.kp >= 0.0f && s.kp <= 100.0f);
    __CPROVER_assume(s.ki >= 0.0f && s.ki <= 10.0f);
    __CPROVER_assume(s.kd >= 0.0f && s.kd <= 10.0f);
    __CPROVER_assume(s.integ_min <= s.integ_max);
    __CPROVER_assume(s.integ >= s.integ_min && s.integ <= s.integ_max);
    
    float err, dt;
    __CPROVER_assume(dt > 0.0f && dt < 0.1f);
    __CPROVER_assume(err >= -1000.0f && err <= 1000.0f);
    
    float out = pid_step(&s, err, dt);
    
    // Properties to prove
    assert(s.integ >= s.integ_min);  // Integral lower bound
    assert(s.integ <= s.integ_max);  // Integral upper bound
    assert(out == out);  // Not NaN
    assert(!isinf(out));  // Not infinite
}
```

#### Run CBMC

```bash
# Run CBMC
cbmc --function harness libraries/AC_PID/AC_PID.cpp \
    --bounds-check \
    --overflow-check \
    --nan-check \
    --unwind 10

# With contract verification
cbmc --function harness libraries/AC_PID/AC_PID.cpp \
    --contracts \
    --show-properties
```

---

## 5. Campaign Structure

### Directory Layout

```
ArduPilot/
├── Tools/
│   ├── autotest/
│   │   ├── test_property_faults.py      # Hypothesis tests
│   │   ├── scenarios/                    # Scenario schemas
│   │   │   ├── copter_rtl.yaml
│   │   │   └── plane_emergency.yaml
│   │   └── fault_injection/              # Fault definitions
│   │       ├── gps_faults.py
│   │       └── link_faults.py
│   └── fuzzing/
│       ├── afl_targets/                  # AFL++ targets
│       │   ├── fuzz_mission_parser.c
│       │   └── fuzz_param_parser.c
│       ├── seeds/                        # Seed corpora
│       └── output/                       # Fuzzing results
├── formal/
│   ├── tla/                              # TLA+ specs
│   │   └── ModeManager.tla
│   ├── lustre/                           # Kind 2 specs
│   │   └── sensor_voter.lustre
│   └── cbmc/                             # CBMC proofs
│       └── pid_controller.c
├── .clang-tidy                           # clang-tidy config
├── .cppcheck                             # Cppcheck config
└── SECURITY_HARDENING.md                 # This document
```

### CI Pipeline

```yaml
# .github/workflows/security-hardening.yml
name: Security Hardening

on:
  push:
    branches: [master]
  pull_request:
  schedule:
    - cron: '0 2 * * *'  # Daily at 2 AM

jobs:
  static-analysis:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run clang-tidy
        run: |
          run-clang-tidy -p build -checks='bugprone-*,cert-*'
      - name: Run Cppcheck
        run: |
          cppcheck --enable=all --error-exitcode=1 src/

  property-testing:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run Hypothesis tests
        run: |
          pip3 install hypothesis pytest
          python3 -m pytest Tools/autotest/test_property_faults.py \
            --hypothesis-examples=500

  fuzzing:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run AFL++
        run: |
          timeout 1800 afl-fuzz -i seeds -o output ./fuzz_parser @@
      - name: Upload crashes
        uses: actions/upload-artifact@v3
        if: always()
        with:
          name: afl-crashes
          path: output/crashes/

  formal-verification:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run CBMC
        run: |
          cbmc --function harness formal/cbmc/pid_controller.c \
            --bounds-check --overflow-check --nan-check
```

### Metrics and Goals

| Metric | Target | Measurement |
|--------|--------|-------------|
| Property test coverage | 10,000+ examples/run | Hypothesis stats |
| Fuzzing coverage | 90%+ block coverage | AFL++ coverage |
| Static analysis | 0 critical findings | clang-tidy/Cppcheck |
| Formal proofs | All critical properties proved | CBMC/Kind 2 output |
| Fault injection success | 95%+ scenarios pass | SITL test results |

---

## References

- [Hypothesis Documentation](https://hypothesis.readthedocs.io/)
- [AFL++ Documentation](https://github.com/AFLplusplus/AFLplusplus)
- [TLA+ Home](https://lamport.azurewebsites.net/tla/tla.html)
- [Kind 2 Model Checker](https://kind2-mc.github.io/)
- [CBMC](https://www.cprover.org/cbmc/)
- [Frama-C](https://frama-c.com/)
- [clang-tidy](https://clang.llvm.org/extra/clang-tidy/)
- [Cppcheck](http://cppcheck.sourceforge.net/)

---

## Contributing

To add new property tests:

1. Create scenario in `Tools/autotest/scenarios/`
2. Define properties in `test_property_faults.py`
3. Run with `pytest --hypothesis-examples=1000`

To add new fuzzing targets:

1. Create harness in `Tools/fuzzing/afl_targets/`
2. Add seed corpus to `Tools/fuzzing/seeds/`
3. Run AFL++ for 24+ hours

To add formal specs:

1. Create TLA+/Lustre/CBMC spec in `formal/`
2. Document properties being proved
3. Add to CI pipeline
