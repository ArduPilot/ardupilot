#pragma once

/*
 * AP_ErrorHandling.h - Common error handling patterns and utilities
 *
 * This file provides standardized macros and patterns for error handling
 * throughout the ArduPilot codebase to improve code quality and consistency.
 */

#include <AP_HAL/AP_HAL.h>

/**
 * @brief Check for nullptr and return early if found
 * 
 * Usage: CHECK_NULLPTR_RET(pointer, return_value);
 * 
 * This macro provides a standardized way to check for nullptr and return
 * with a specific value if the pointer is null. It includes debug logging
 * when compiled with HAL_DEBUG_BUILD.
 */
#define CHECK_NULLPTR_RET(ptr, ret_val) \
    do { \
        if ((ptr) == nullptr) { \
            DEV_PRINTF("Nullptr check failed: " #ptr " at %s:%d\n", __FILE__, __LINE__); \
            return ret_val; \
        } \
    } while(0)

/**
 * @brief Check for nullptr and return void if found
 * 
 * Usage: CHECK_NULLPTR(pointer);
 * 
 * Similar to CHECK_NULLPTR_RET but for void functions.
 */
#define CHECK_NULLPTR(ptr) \
    do { \
        if ((ptr) == nullptr) { \
            DEV_PRINTF("Nullptr check failed: " #ptr " at %s:%d\n", __FILE__, __LINE__); \
            return; \
        } \
    } while(0)

/**
 * @brief Check boolean condition and return early if false
 * 
 * Usage: CHECK_CONDITION_RET(condition, return_value);
 * 
 * Provides a standardized way to check conditions with debug logging.
 */
#define CHECK_CONDITION_RET(cond, ret_val) \
    do { \
        if (!(cond)) { \
            DEV_PRINTF("Condition check failed: " #cond " at %s:%d\n", __FILE__, __LINE__); \
            return ret_val; \
        } \
    } while(0)

/**
 * @brief Verify function return value and propagate errors
 * 
 * Usage: VERIFY_CALL(function_call(), error_return_value);
 * 
 * This macro checks if a function call returns false (indicating error)
 * and propagates the error by returning early.
 */
#define VERIFY_CALL(call, ret_val) \
    do { \
        if (!(call)) { \
            DEV_PRINTF("Call failed: " #call " at %s:%d\n", __FILE__, __LINE__); \
            return ret_val; \
        } \
    } while(0)

/**
 * @brief Safe division with zero-check
 * 
 * Usage: float result = SAFE_DIV(numerator, denominator, default_value);
 * 
 * Returns default_value if denominator is zero or very close to zero.
 */
template<typename T>
inline T safe_div(T num, T denom, T default_val = 0) {
    if (is_zero(denom)) {
        return default_val;
    }
    return num / denom;
}

/**
 * @brief RAII guard for ensuring cleanup happens on scope exit
 * 
 * Usage: ScopeGuard guard([&]() { cleanup_code(); });
 * 
 * Ensures cleanup code runs even if early returns or exceptions occur.
 */
class ScopeGuard {
public:
    template<typename Func>
    explicit ScopeGuard(Func&& func) : cleanup(std::forward<Func>(func)), active(true) {}
    
    ~ScopeGuard() {
        if (active) {
            cleanup();
        }
    }
    
    // Disable copy
    ScopeGuard(const ScopeGuard&) = delete;
    ScopeGuard& operator=(const ScopeGuard&) = delete;
    
    // Allow dismissal if cleanup not needed
    void dismiss() { active = false; }
    
private:
    std::function<void()> cleanup;
    bool active;
};

// Helper macro to create named scope guards
#define SCOPE_GUARD_CONCAT_IMPL(x, y) x##y
#define SCOPE_GUARD_CONCAT(x, y) SCOPE_GUARD_CONCAT_IMPL(x, y)
#define SCOPE_EXIT(code) ScopeGuard SCOPE_GUARD_CONCAT(scope_guard_, __LINE__)([&]() { code; })
