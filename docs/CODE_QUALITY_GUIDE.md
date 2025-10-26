# ArduPilot Code Quality Guide

## Overview
This document provides guidelines for maintaining high code quality throughout the ArduPilot codebase, with a focus on memory management, error handling, and reducing technical debt.

## Memory Management Best Practices

### 1. Prefer RAII (Resource Acquisition Is Initialization)
RAII ensures resources are automatically cleaned up when they go out of scope.

**Good:**
```cpp
void process_data() {
    ScopeGuard cleanup([&]() {
        // Cleanup happens automatically
        release_resources();
    });
    
    // Process data - cleanup happens even on early return
    if (error_condition) {
        return;  // cleanup() still called
    }
}
```

**Avoid:**
```cpp
void process_data() {
    allocate_resources();
    if (error_condition) {
        return;  // Resource leak!
    }
    release_resources();
}
```

### 2. Check All Allocations
Always verify that memory allocation succeeded before using the allocated memory.

**Good:**
```cpp
const char* copy = strdup(original);
if (copy == nullptr) {
    // Handle allocation failure
    return false;
}
// Use copy safely
```

**Avoid:**
```cpp
const char* copy = strdup(original);
// Directly use copy without checking
process_string(copy);  // Potential nullptr dereference
```

### 3. Match Allocation/Deallocation Methods
- `new` → `delete`
- `new[]` → `delete[]`
- `malloc()` → `free()`
- `strdup()` → `free()`

**Never mix allocation and deallocation methods!**

### 4. Initialize Pointers
Always initialize pointers to nullptr to avoid dangling pointer issues.

```cpp
MyClass* ptr = nullptr;  // Good
// vs
MyClass* ptr;  // Dangerous - uninitialized
```

## Error Handling Best Practices

### 1. Consistent Error Return Checking
Always check return values from functions that can fail.

**Good:**
```cpp
if (!ahrs.get_velocity_NED(vel_ned)) {
    // Handle the error appropriately
    gcs().send_text(MAV_SEVERITY_WARNING, "Failed to get velocity");
    return false;
}
```

**Avoid:**
```cpp
ahrs.get_velocity_NED(vel_ned);  // Ignoring potential failure
```

### 2. Use Helper Macros for Consistency
The `AP_ErrorHandling.h` header provides standardized macros:

```cpp
#include <AP_Common/AP_ErrorHandling.h>

bool MyClass::process() {
    CHECK_NULLPTR_RET(important_ptr, false);
    VERIFY_CALL(initialize(), false);
    
    // Rest of function
    return true;
}
```

### 3. Fail Early and Clearly
Validate inputs and preconditions at the start of functions.

```cpp
bool set_target(const Vector3f& pos) {
    // Validate early
    if (!is_initialized()) {
        return false;
    }
    
    if (!pos.is_finite()) {
        return false;
    }
    
    // Proceed with validated data
    _target = pos;
    return true;
}
```

### 4. Provide Meaningful Error Context
When logging errors, include enough context for debugging.

```cpp
if (velocity > max_velocity) {
    gcs().send_text(MAV_SEVERITY_WARNING, 
                    "Velocity %.1f exceeds max %.1f", 
                    velocity, max_velocity);
    return false;
}
```

## Technical Debt Management

### 1. TODO Comment Guidelines
- **DO**: Provide context and reference GitHub issues
- **DO**: Include estimates of complexity
- **DON'T**: Leave TODO comments for years without action
- **DON'T**: Use TODO for architectural decisions

**Good:**
```cpp
// Feature tracked in issue #12345: Add support for dynamic offset adjustment
// Requires parameter framework extension and UI updates
```

**Avoid:**
```cpp
// TODO: make this better
// TODO: fix this
```

### 2. Prioritize Technical Debt

| Priority | Type | Action Timeline |
|----------|------|-----------------|
| Critical | Safety-related issues, memory leaks in common paths | Immediate |
| High | Error handling gaps, inconsistent patterns in core modules | Within 1 release |
| Medium | Code duplication, minor optimization opportunities | Within 3 releases |
| Low | Style inconsistencies, documentation gaps | Ongoing |

### 3. Refactoring Guidelines
- Make changes incrementally
- Add tests before refactoring
- Maintain backward compatibility where possible
- Document breaking changes clearly

## Code Review Checklist

When reviewing code, check for:

- [ ] All memory allocations are checked
- [ ] No memory leaks in error paths
- [ ] Consistent error handling throughout
- [ ] No dangling pointers
- [ ] RAII used where appropriate
- [ ] Return values checked for all fallible functions
- [ ] TODO comments are actionable and tracked
- [ ] Code follows existing patterns in the module
- [ ] Adequate logging for debugging
- [ ] Edge cases handled appropriately

## Null Pointer Safety

### 1. Use nullptr checks consistently
```cpp
AP_Mount* mount = AP_Mount::get_singleton();
if (mount != nullptr) {
    mount->set_target(target);
}
```

### 2. Prefer references over pointers when not-null
```cpp
// If parameter must not be null, use reference
void process(const Vector3f& position) {
    // No null check needed
}

// If parameter can be null, use pointer
void process(const Vector3f* position) {
    if (position == nullptr) {
        return;
    }
}
```

## Exception Safety

While ArduPilot generally doesn't use exceptions (disabled in most builds), write code with strong exception safety guarantees:

1. **Basic guarantee**: If an error occurs, no resources are leaked
2. **Strong guarantee**: If an error occurs, program state is unchanged
3. **No-throw guarantee**: Operations always succeed

```cpp
bool MyClass::update_state(const State& new_state) {
    // Create backup before modification (strong guarantee)
    State backup = current_state;
    
    // Attempt update
    if (!validate(new_state)) {
        // State unchanged on error
        return false;
    }
    
    current_state = new_state;
    return true;
}
```

## Performance Considerations

### 1. Avoid Allocations in Hot Paths
- Pre-allocate buffers during initialization
- Use stack allocation for small, fixed-size objects
- Pool frequently allocated objects

### 2. Minimize Pointer Indirection
```cpp
// Good for frequent access
struct Controller {
    Vector3f target;  // Direct member
};

// Avoid if not necessary
struct Controller {
    Vector3f* target;  // Extra indirection
};
```

## Testing

### 1. Test Error Paths
Don't just test the happy path:

```cpp
// Test allocation failure
TEST(RegisterMode, AllocationFailure) {
    // Simulate low memory condition
    // Verify graceful degradation
}
```

### 2. Test Boundary Conditions
- Null pointers
- Zero values
- Maximum values
- Empty containers

## Migration Strategy

### Phase 1: Critical Fixes (Immediate)
- Fix memory leaks in register_custom_mode()
- Add nullptr checks in crash-prone paths
- Document unclear TODO items

### Phase 2: Standardization (Next Release)
- Introduce AP_ErrorHandling.h utilities
- Apply to new code immediately
- Gradually refactor existing code

### Phase 3: Systematic Cleanup (Ongoing)
- Address TODO items by priority
- Improve consistency across modules
- Enhance test coverage

## Resources

- [ArduPilot Coding Style](https://ardupilot.org/dev/docs/coding-style.html)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/)
- RAII and Smart Pointers in embedded systems

## Questions?

For questions about code quality or these guidelines, reach out on:
- ArduPilot Discord
- Developer mailing list
- GitHub discussions
