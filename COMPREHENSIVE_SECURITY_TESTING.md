# ArduPilot Comprehensive Security Testing Framework

This document describes the complete multi-layer security testing approach for ArduPilot.

## Layer 1: Property-Based Testing with Hypothesis

### Overview

Hypothesis generates thousands of test cases automatically from property definitions, finding edge cases that manual testing misses.

### Installation

```bash
pip3 install hypothesis pytest pytest-timeout
```

### Fault Injection Types

#### Timing Faults
- Slow sensor update
- Delayed command loop  
- Skipped scheduler tick

#### Sensor Faults
- GPS jump (position discontinuity)
- GPS freeze (stale data)
- IMU bias ramp
- IMU spike
- Barometer drift
- Magnetometer spike

#### Link Faults
- MAVLink delay
- Packet loss
- Bandwidth limitation

#### Data Corruption
- Mission file corruption
- Malformed parameter values
- Bad fence data

### Scenario Schema (YAML)

```yaml
vehicle: copter
map: windy_coast
mission: rtl_after_wp3
faults:
  - type: gps_jump
    t_sec: 42
    dx_m: 35
    dy_m: -10
  - type: imu_bias_ramp
    start_t_sec: 55
    axis: z
    rate_dps_per_sec: 0.8
  - type: link_impairment
    start_t_sec: 30
    duration_sec: 20
    netem:
      delay_ms: 250
      loss_pct: 15
success_properties:
  - no_disarm_in_air
  - no_geofence_breach
  - mode_eventually_in: [RTL, LAND, LOITER]
  - max_attitude_error_deg_lt: 35
```

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
    scenario = {
        "faults": [
            {"type": "gps_jump", "t_sec": t_sec, "dx_m": dx, "dy_m": dy},
            {"type": "link_impairment", "start_t_sec": max(1, t_sec - 3),
             "duration_sec": 15, "netem": {"loss_pct": loss}}
        ]
    }
    result = run_sitl_scenario(scenario)
    assert result.no_in_air_disarm
    assert result.mode_end in {"RTL", "LAND", "LOITER"}
    assert result.max_roll_deg < 60
```

### Running Tests

```bash
cd Tools/autotest
python3 -m pytest test_property_faults.py -v --hypothesis-examples=5000
```

---

## Layer 2: AFL++ Fuzzing

### Overview

AFL++ is a coverage-guided fuzzer that finds edge-case bugs in input parsers by generating malformed inputs.

### Installation

```bash
# Ubuntu/Debian
sudo apt-get install afl++ llvm

# Or build from source
git clone https://github.com/AFLplusplus/AFLplusplus
cd AFLplusplus
make distrib-install
```

### Build for Fuzzing

```bash
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

---

## Layer 3: Static Analysis Pipeline

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

### Recommended Policy

#### Hard fail in CI
- Compiler warnings as errors for safety-critical subset
- clang-tidy checks for:
  - narrowing conversions
  - implicit sign conversions
  - unchecked return values
  - suspicious memory ownership
  - bugprone and cert-like checks
- Clang Static Analyzer path-sensitive findings
- Cppcheck findings at high severity

#### Ban list
- Dynamic allocation in flight-critical loops
- Unbounded recursion
- Exceptions in embedded-critical code
- Non-const globals in control logic
- Floating-point comparisons without tolerance

#### Advisory initially, hard fail later
- Cppcheck MISRA addon
- Naming/style consistency
- Dead code
- Complexity thresholds

### Make Standards Tractable

Do NOT try to make all ArduPilot MISRA-clean at once. Instead:

1. **Define a critical subset:**
   - Attitude controller
   - Rate controller
   - Mode transitions
   - Failsafe decision logic
   - Estimator interface wrappers

2. **Put only that subset under strictest policy first**

3. **Require all new code in that subset to be clean**

4. **Gradually ratchet legacy violations downward**

---

## Layer 4: Formal Verification

### Level A: TLA+ for State Machines

Use TLA+ for discrete safety logic and mode-management rules, not continuous control math.

#### Installation

```bash
# Install TLAPS (TLA+ Proof System)
# See https://tlapl.us/ for installation instructions

# Install Apalache for symbolic checking
curl -L https://github.com/informalsystems/apalache/releases/latest/download/apalache-linux.tar.gz | tar xz
```

#### Good TLA+ Targets

- Arming/disarming state machine
- Mode transition rules
- Failsafe priority rules
- "Once condition X persists for N cycles, mode must become RTL/LAND"
- Mutual exclusion and sequencing in mission/failsafe logic

#### Example Properties

- Never arm when pre-arm checks fail
- Never command AUTO and LAND simultaneously
- If GPS becomes invalid for long enough, vehicle eventually leaves GPS-dependent mode
- Failsafe priority ordering is deterministic

#### Run Model Checker

```bash
# TLC model checker
java -jar tla2tools.jar -workers 4 ModeManager.tla

# Apalache symbolic checking
apalache-mc check --inv=EventualSafety ModeManager.tla
```

### Level B: Kind 2 for Synchronous Logic

Use Kind 2 if you can express controller decision logic as synchronous reactive blocks in Lustre.

#### Installation

```bash
# Kind 2 model checker
git clone https://github.com/kind2-mc/kind2.git
cd kind2
make
```

#### Good Kind 2 Targets

- Mode gating logic
- Watchdog escalation logic
- Sensor-validity voting logic
- Finite-state fault manager

#### Run Kind 2

```bash
kind2 --input lustre voter.lustre --smt_solver z3
```

### Level C: CBMC for Code Proofs

Use CBMC and Frama-C on small C kernels.

#### Installation

```bash
# CBMC
sudo apt-get install cbmc

# Or build from source
git clone https://github.com/diffblue/cbmc.git
cd cbmc
make
```

#### Good Candidates

- Attitude/rate controller math helpers
- Saturation and limiting logic
- Anti-windup logic
- Quaternion/rotation conversion helpers
- Voter logic for redundant sensors
- Geofence predicate evaluation
- Failsafe decision function

#### Example CBMC Proof

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

float pid_step(pid_t *s, float err, float dt);

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

## Layer 5: Code Architecture for Verification

### Three Code Zones

#### Zone 1: Proven Core
- Pure C/C++ functions
- No heap
- Minimal dependencies
- Strict coding standard
- CBMC/Frama-C eligible

#### Zone 2: Heavily Tested Control Integration
- SITL/HITL tested
- Strict static analysis
- Property-based and fault-injection coverage

#### Zone 3: Peripheral/Support Code
- Parsers, logs, GCS extras, scripts
- Fuzzed and statically analyzed
- Lower formal burden

### How to Choose What to Prove

Pick components with all of these traits:
- Small API
- Limited hidden state
- No I/O
- Deterministic
- No dynamic allocation
- Easy to stub dependencies

**Best payoff:**
- No NaNs or infinities can escape critical math helpers under stated assumptions
- Integrators stay bounded
- Saturations always hold
- Voter outputs remain within valid index/range
- Failsafe state machine respects precedence and liveness

---

## Continuous Integration

### GitHub Actions Workflow

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

---

## Metrics and Goals

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
