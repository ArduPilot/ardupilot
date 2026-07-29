# ArduPilot Coding Standards

This document consolidates all formatting rules enforced by tooling (astyle,
pre-commit, CI) and the style conventions regularly enforced during code review.
The goal is to eliminate recurring review comments by making the rules explicit
and discoverable.

**Canonical references:**

- [Style Guide (wiki)](https://ardupilot.org/dev/docs/style-guide.html)
- [Submitting Patches](https://ardupilot.org/dev/docs/submitting-patches-back-to-master.html)
- `Tools/CodeStyle/astylerc` — astyle configuration
- `.editorconfig` — editor integration
- `.pre-commit-config.yaml` — pre-commit hooks

---

## Table of Contents

1. [C++ Formatting (astyle rules)](#1-c-formatting-astyle-rules)
2. [Braces](#2-braces)
3. [Spacing](#3-spacing)
4. [Naming Conventions](#4-naming-conventions)
5. [Types and Literals](#5-types-and-literals)
6. [Comments and Documentation](#6-comments-and-documentation)
7. [Parameters](#7-parameters)
8. [Memory and Performance](#8-memory-and-performance)
9. [Common Review Feedback](#9-common-review-feedback)
10. [Commit Conventions](#10-commit-conventions)
11. [Python Standards](#11-python-standards)
12. [Tooling Quick Reference](#12-tooling-quick-reference)

---

## 1. C++ Formatting (astyle rules)

The project uses [Artistic Style (astyle)](http://astyle.sourceforge.net/) with
the configuration in `Tools/CodeStyle/astylerc`:

| astylerc directive           | Meaning |
|------------------------------|---------|
| `style=linux`                | Linux brace style (see [Braces](#2-braces) below) |
| `indent=spaces=4`            | 4 spaces per indent level. **Never tabs.** |
| `add-braces`                 | Always add braces to single-statement bodies (`if`, `for`, `while`, `else`). |
| `keep-one-line-statements`   | Permit single-statement bodies on the same line as the control keyword. |
| `indent-col1-comments`       | Indent comments that start in column 1. |
| `min-conditional-indent=0`   | No extra indent for conditional continuation lines. |
| `pad-header`                 | Space between control keywords and parentheses: `if (x)` not `if(x)`. |
| `lineend=linux`              | LF line endings. No CRLF. |
| `suffix=none`                | No backup files created. |

### Running astyle

```bash
# Check formatting (dry run — reports but does not change files):
astyle --options=Tools/CodeStyle/astylerc --dry-run path/to/file.cpp

# Apply formatting:
astyle --options=Tools/CodeStyle/astylerc path/to/file.cpp

# Or use the wrapper script:
Tools/CodeStyle/ardupilot-astyle.sh path/to/file.cpp
```

> **Important:** Only run astyle on lines/files **you have modified**. Running it
> on entire unmodified files creates noise in the diff and will not be accepted.

### Directories enforced by CI

The following paths are checked for astyle compliance on every PR
(see `Tools/scripts/run_astyle.py`):

- `libraries/AP_DDS/`
- `libraries/AP_ExternalControl/`
- `libraries/AP_GSOF/`
- `ArduCopter/AP_ExternalControl_Copter.{cpp,h}`
- `Rover/AP_ExternalControl_Rover.{cpp,h}`
- `libraries/AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain7.{cpp,h}`

All other code should follow the same rules — it just isn't yet enforced
automatically.

---

## 2. Braces

### Function definitions — each brace on its own line

```cpp
// Correct:
void Copter::init_ardupilot()
{
    setup_frame();
}

// Wrong:
void Copter::init_ardupilot() {
    setup_frame();
}
```

### Control statements — opening brace on the preceding line

The `style=linux` setting in astyle places the opening brace of control
statements (`if`, `for`, `while`, `switch`, `do`) on the same line as the
keyword:

```cpp
// Correct:
if (condition) {
    do_something();
}

// Wrong:
if (condition)
{
    do_something();
}
```

### `else` goes on the same line as the closing brace

```cpp
// Correct:
if (condition) {
    foo();
} else {
    bar();
}

// Wrong:
if (condition) {
    foo();
}
else {
    bar();
}
```

### Always use braces — even for single statements

The `add-braces` directive means every `if`, `for`, `while`, and `else` body
must be enclosed in braces, even if it is a single statement:

```cpp
// Correct:
if (done) {
    return;
}

// Wrong:
if (done)
    return;
```

### Class/struct/namespace — opening brace on its own line

```cpp
class MyClass
{
public:
    void method();
};
```

### Short inline methods in headers may use a single line

```cpp
bool is_enabled() const { return _enabled; }
```

---

## 3. Spacing

### Control keywords — space before parenthesis

```cpp
if (x)          // ✓
for (int i …)   // ✓
while (running)  // ✓
switch (mode)    // ✓

if(x)           // ✗
for(int i …)    // ✗
```

### Function calls — no space before parenthesis

```cpp
foo(a, 10);     // ✓
foo (a, 10);    // ✗
```

### Unary operators — no spaces

```cpp
i++;    // ✓
!flag   // ✓
-value  // ✓

i ++;   // ✗
! flag  // ✗
```

### Trailing whitespace

Remove trailing whitespace from any line you touch. Check with:

```bash
git diff --check HEAD
```

### No horizontal alignment padding

Do not add extra spaces to visually align `=` signs, comments, or values across
lines. It creates churn when a longer name is added later.

```cpp
// Correct:
int x = 1;
int longer_name = 2;

// Discouraged:
int x           = 1;
int longer_name = 2;
```

---

## 4. Naming Conventions

### General rules

| Element | Convention | Example |
|---------|-----------|---------|
| Variables, functions | `lower_snake_case` | `get_altitude_cm()` |
| Classes | `PascalCase` (may use underscores between words) | `AP_Compass`, `AC_PosControl` |
| Enum classes | `PascalCase`, **singular** | `enum class FlightMode` |
| Enum values | `PascalCase` | `FlightMode::Loiter` |
| Constants / defines | `UPPER_SNAKE_CASE` | `MAX_CHANNELS` |
| Private members | Optional leading underscore | `_enabled`, `enabled` |
| Parameters | `UPPER_SNAKE_CASE`, most important word first | `RTL_ALT_MIN` |

### Unit suffixes are mandatory

Every variable or function that represents a physical quantity must include a
unit suffix. This is one of the most frequently enforced review rules.

| Quantity | Suffix | Example |
|----------|--------|---------|
| Distance (meters) | `_m` | `altitude_m` |
| Distance (centimeters) | `_cm` | `altitude_cm` |
| Distance (millimeters) | `_mm` | `offset_mm` |
| Angle (radians) | `_rad` | `heading_rad` |
| Angle (degrees) | `_deg` | `heading_deg` |
| Angle (centidegrees) | `_cdeg` | `heading_cdeg` |
| Angular rate (rad/s) | `_rads` | `yaw_rate_rads` |
| Angular rate (deg/s) | `_dps` | `roll_rate_dps` |
| Velocity (m/s) | `_ms` | `airspeed_ms` |
| Velocity (cm/s) | `_cms` | `climb_rate_cms` |
| Acceleration (m/s²) | `_mss` | `accel_mss` |
| Time (seconds) | `_s` | `timeout_s` |
| Time (milliseconds) | `_ms` | `last_update_ms` |
| Time (microseconds) | `_us` | `pulse_width_us` |
| Temperature | `_degc` | `board_temp_degc` |
| Current | `_amp` | `battery_current_amp` |
| Frequency | `_hz` | `sample_rate_hz` |

When the variable name unambiguously implies the canonical SI unit (seconds,
meters, degrees, °C), the suffix may be omitted — but when in doubt, add it.

---

## 5. Types and Literals

### Use `enum class`, not plain `enum`

```cpp
// Correct:
enum class FlightMode : uint8_t {
    Stabilize = 0,
    Acro = 1,
};

// Wrong:
enum FlightMode {
    FLIGHT_MODE_STABILIZE = 0,
    FLIGHT_MODE_ACRO = 1,
};
```

### Float literals

ArduPilot is compiled with `-fsingle-precision-constant`, so bare `1.0`
is treated as float rather than double. The `f` suffix is **permitted and
encouraged** for clarity but not strictly required:

```cpp
float x = 1.0f;   // preferred — explicit
float x = 1.0;    // acceptable — compiler flag handles it
```

### Prefer multiplication over division

Multiplication is faster and avoids division-by-zero risk:

```cpp
alt_m = alt_cm * 0.01f;    // ✓
alt_m = alt_cm / 100.0f;   // ✗ (slower, less safe)
```

### Use `static_cast`, not C-style casts

```cpp
uint32_t val = static_cast<uint32_t>(payload[1]) << 8;   // ✓
uint32_t val = (uint32_t)payload[1] << 8;                 // ✗
```

### Use `nullptr`, not `NULL` or `0`

```cpp
if (ptr == nullptr) { … }   // ✓
if (ptr == NULL) { … }      // ✗
if (ptr == 0) { … }         // ✗
```

### No standard library containers

Do not use `std::vector`, `std::string`, `std::map`, `std::unordered_map`, etc.
Use fixed-size arrays, custom structures, or AP-specific alternatives. The
standard library pulls in too much code and causes unpredictable heap usage.

### Use `ARRAY_SIZE()` for array lengths

```cpp
for (uint8_t i = 0; i < ARRAY_SIZE(channels); i++) {   // ✓
for (uint8_t i = 0; i < 16; i++) {                      // ✗ magic number
```

### Use `is_zero()`, `is_positive()`, `is_negative()` for float comparisons

```cpp
if (is_zero(value)) { … }       // ✓
if (value == 0.0f) { … }        // ✗ (float equality is unreliable)

if (is_positive(gain)) { … }    // ✓
if (gain > 0.0f) { … }          // acceptable in some contexts
```

---

## 6. Comments and Documentation

### Every file needs a header

New `.h` and `.cpp` files must begin with the GPLv3 license block and a brief
description of the file's purpose.

### Every function declaration needs a comment

```cpp
// return altitude above home in centimeters
int32_t get_altitude_cm() const;
```

### Comments explain "why", not "what"

```cpp
// ✓ Good: explain the reasoning
// Clamp to avoid oscillation when close to waypoint
if (dist_m < 0.5f) {

// ✗ Bad: just restates the code
// Check if dist_m is less than 0.5
if (dist_m < 0.5f) {
```

### No commented-out code

Dead code must be removed, not commented out. Git history preserves everything.

```cpp
// ✗ Do not do this:
// old_function();
// if (old_condition) { ... }
```

### No process/development comments

Comments must describe code operation, not development history or TODOs for
the PR author:

```cpp
// ✗ Bad:
// Added by Andy 2024-03-15
// TODO: fix this later
// HACK: workaround for issue #1234
```

---

## 7. Parameters

### Required documentation block

Every `AP_Param` parameter must have a complete documentation block **directly
above** the parameter declaration:

```cpp
// @Param: RTL_ALT
// @DisplayName: RTL Altitude
// @Description: The minimum altitude the vehicle will move to before returning to launch.
// @User: Standard
// @Units: cm
// @Range: 200 8000
// @Increment: 1
AP_Int16 rtl_alt;
```

### Naming rules

- Maximum **16 characters**
- `UPPER_SNAKE_CASE`
- Most important word first: `RTL_ALT_MIN` not `MIN_RTL_ALT`
- Reuse existing parameter vocabulary when possible

### Description rules

- Present tense
- Explain consequences of changing the value
- Explain when the parameter is used and when it is ignored
- Do not repeat the display name as the description

---

## 8. Memory and Performance

### No dynamic allocation in flight-critical paths

Do not use `malloc`, `calloc`, or `new` in code that runs during flight.
Allocate during `init()` or use statically-sized buffers.

### `new` and `malloc` zero their memory; stack variables do not

```cpp
auto *obj = new MyClass();  // memory is zeroed
MyClass stack_obj;           // members are NOT zeroed — initialize them
```

### Prefer `calloc`/`free` over `new[]`/`delete[]` for arrays

```cpp
auto *leds = (SerialLed *)calloc(num_leds, sizeof(SerialLed));  // ✓
free(leds);

auto *leds = new SerialLed[num_leds];  // ✗ more overhead
delete[] leds;
```

### Prefer embedded objects over heap allocation

```cpp
class Parent {
    Child _child;    // ✓ embedded, no heap allocation
};

class Parent {
    Child *_child;   // ✗ requires new/delete when lifetime matches parent
};
```

### No `printf`

Use `GCS_SEND_TEXT()` for messages to the ground station. Use
`hal.console->printf()` only for debug code that is compiled out by default.

### Avoid deep recursion and large stack variables

Stack space is limited on embedded targets (often 1-2 KB per task).

### Bit fields are discouraged

The flash savings rarely justify the increased code size from bit manipulation.
Prefer `bool` members.

---

## 9. Common Review Feedback

These are patterns **repeatedly flagged during code review** that are not
covered by the formatting/naming/parameter sections above. Addressing these
before submitting a PR will significantly speed up the review process.

### Code structure

| Issue | Fix |
|-------|-----|
| Missing `const` on methods that don't modify state | Add `const` qualifier |
| Deep nesting | Return early to reduce indentation levels |
| Unused `#include` | Remove includes that aren't needed |
| `using namespace` in headers | Never. Pollutes every file that includes the header. |

### API and architecture

| Issue | Fix |
|-------|-----|
| Calling an API without verifying its signature | Check the header file first |
| Missing `WITH_SEMAPHORE` for shared data | Use the semaphore macros for thread-safe access |
| Using `AP_HAL::millis()` wrong | Returns `uint32_t` milliseconds — handle wraparound |
| Platform-specific code in common libraries | Use AP_HAL abstractions |
| Optional feature depending on core code wrong way | Core must never `#include` optional feature headers. Use callbacks or interfaces. |
| Missing `#pragma once` in new headers | Use `#pragma once` (preferred over include guards) |

---

## 10. Commit Conventions

### Message format

```
Subsystem: brief description of the change

Optional longer description explaining why the change is needed,
any non-obvious design decisions, or testing performed.
```

### Subsystem prefix rules

| Files changed | Prefix |
|--------------|--------|
| `libraries/AP_GPS/…` | `AP_GPS:` |
| `libraries/AC_PosControl/…` | `AC_PosControl:` |
| `ArduCopter/…` | `Copter:` |
| `ArduPlane/…` | `Plane:` |
| `Rover/…` | `Rover:` |
| `ArduSub/…` | `Sub:` |
| `Tools/autotest/…` | `autotest:` |
| `Tools/scripts/…` | `scripts:` |
| `Tools/AP_Bootloader/…` | `AP_Bootloader:` |
| `Tools/bootloaders/…` | `bootloaders:` |
| `Tools/ardupilotwaf/…` | `ardupilotwaf:` |
| `libraries/AP_HAL_ChibiOS/hwdef/…` | `AP_HAL_ChibiOS:` |

### Rules enforced by CI

These are checked automatically by `Tools/scripts/check_branch_conventions.py`:

- No merge commits
- No `fixup!` commits
- Every commit subject has a subsystem prefix before `:`
- Prefix contains only: letters, digits, `.`, `_`, `/`, `-`, spaces, quotes
- Subject line ≤ 160 characters
- Changed `.md` files pass `markdownlint-cli2`

### Commit organization

- **One logical change per commit** — a reviewer should be able to understand
  each commit independently
- **One subsystem per commit** — if a feature requires changes to `AP_GPS` and
  `Copter`, that is two commits
- **Every commit must compile** — do not break the build between commits
- **No formatting-only commits** — the disruption to `git blame` is not worth it
- **Do not clean up whitespace on lines you didn't change** — it makes the diff
  harder to review

---

## 11. Python Standards

### Tools used

| Tool | Config file | Purpose |
|------|------------|---------|
| flake8 | `.flake8` | Style guide enforcement (max line length: 127) |
| ruff | `pyproject.toml` | Linting (target: Python 3.10+) |
| black | `pyproject.toml` | Formatting for `AP_DDS/` and `Tools/ros2/` (line length: 120) |
| isort | `pyproject.toml` | Import sorting (force single-line, black profile) |
| codespell | `.pre-commit-config.yaml` | Spell checking |

### Flake8 opt-in

Python files that are "flake8-clean" contain the marker comment:

```python
# AP_FLAKE8_CLEAN
```

Adding this marker to a file opts it into CI checking. New Python files should
include it.

---

## 12. Tooling Quick Reference

### Before committing

```bash
# Check for trailing whitespace:
git diff --check HEAD

# Check astyle formatting (dry run):
astyle --options=Tools/CodeStyle/astylerc --dry-run <modified-files>

# Run pre-commit hooks:
pre-commit run --files <modified-files>
```

### Editor setup

Install an [EditorConfig](https://editorconfig.org/) plugin. The
`.editorconfig` file will automatically set:

- 4-space indent
- LF line endings
- UTF-8 charset
- Final newline insertion

### CI checks that will block your PR

| Check | What it does |
|-------|-------------|
| `astyle-cleanliness` | Runs astyle on enforced directories |
| `python-cleanliness` | Runs flake8 on `AP_FLAKE8_CLEAN` files |
| `pre-commit` | Runs all hooks in `.pre-commit-config.yaml` |
| `check-branch-conventions` | Validates commit messages, no merges, markdown |

---

## Summary

The most common reasons PRs get style comments:

1. **Missing unit suffixes** on variable/function names
2. **Wrong brace placement** (function definitions vs control statements)
3. **Missing braces** on single-statement `if`/`for`/`while`
4. **Trailing whitespace**
5. **Commits touching multiple subsystems**
6. **Missing or wrong subsystem prefix** in commit messages
7. **Commented-out dead code** left in
8. **Missing `const`** on read-only methods
9. **Using `std::` containers** instead of fixed arrays
10. **Missing parameter documentation** blocks

Fix these before submitting and you'll have a much smoother review.
