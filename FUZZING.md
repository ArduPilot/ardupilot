# ArduPilot Fuzzing Framework

This document describes the focused fuzzing framework for ArduPilot security hardening.

## Overview

The fuzzing framework targets high-risk components identified during security review:
- Protocol parsers (AIS, RC protocols)
- Mathematical functions (division, square root)
- Buffer operations (strcpy, memcpy)
- Array indexing operations

## Quick Start

```bash
# Build with fuzzing tests enabled
./waf configure --board sitl --enable-fuzzer
./waf check

# Run specific fuzzing tests
./build/sitl/bin/test_fuzzing
```

## Fuzzing Tests

The fuzzing tests are located in `libraries/AP_Fuzzing/tests/test_fuzzing.cpp` and include:

### 1. AIS Parser Fuzzing

**Target:** `libraries/AP_AIS/AP_AIS.cpp`

**Test:** `FuzzingTest.AISParserRandomInputs`

Generates 1000 random AIS sentences with edge case values to test:
- Multi-part message handling
- Invalid part counts
- Overflow payload sizes
- Malformed checksums

### 2. Math Functions Fuzzing

**Targets:** 
- `libraries/AP_Math/AP_Math.cpp`
- `libraries/AP_AHRS/AP_AHRS_Backend.cpp`
- `libraries/AP_Soaring/Variometer.cpp`

**Tests:**
- `FuzzingTest.MathDivisionByZero` - Tests 10000 random division operations
- `FuzzingTest.MathSafeSqrt` - Tests square root with negative inputs
- `FuzzingTest.VariometerDivisionByZero` - Tests variometer calculations

Generates edge case floating-point values:
- Zero, negative zero
- Infinity, negative infinity
- NaN
- FLT_MIN, FLT_MAX
- Very small/large values

### 3. RC Protocol Fuzzing

**Target:** `libraries/AP_RCProtocol/`

**Test:** `FuzzingTest.SUMDParserRandomFrames`

Generates malformed RC protocol frames:
- Invalid channel counts (0-255)
- Truncated frames
- Invalid checksums
- Oversized payloads

### 4. String Operations Fuzzing

**Target:** All `strcpy`, `strncpy` usage

**Test:** `FuzzingTest.SafeStringCopy`

Tests:
- Strings longer than destination buffers
- Strings without null terminators
- Empty strings

### 5. Array Bounds Fuzzing

**Target:** All array access operations

**Test:** `FuzzingTest.SafeArrayAccess`

Tests boundary indices:
- 0, 1 (lower boundary)
- size-1, size, size+1 (upper boundary)
- 255 (uint8_t max)
