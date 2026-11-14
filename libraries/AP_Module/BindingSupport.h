// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <AP_Module/BindingConfig.h>
#include <AP_HAL/Semaphores.h>
#include <cmath>
#include <limits>
#include <string.h>
#include <type_traits>

namespace AP_Module
{

class BindingLock
{
public:
    explicit BindingLock(AP_HAL::Semaphore &sem) : _sem(sem)
    {
        // AP_HAL defines a zero timeout as an unbounded, blocking take.
        const bool taken = _sem.take(0);
        (void)taken;
    }
    CLASS_NO_COPY(BindingLock);
    ~BindingLock()
    {
        _sem.give();
    }
private:
    AP_HAL::Semaphore &_sem;
};

// The wire format has no alignment requirement and exposes no C++ layouts.
struct BindingWire {
    template <typename T>
    static bool valid_enum(int32_t value)
    {
        // Description ranges must also fit the C++ enum's legal value range;
        // an unfixed enum can have a narrower range than its underlying type.
        using U = typename std::underlying_type<T>::type;
        static_assert(sizeof(U) <= 4, "enum wire supports at most 32-bit underlying types");
        return int64_t(value) >= int64_t(std::numeric_limits<U>::lowest()) &&
               int64_t(value) <= int64_t(std::numeric_limits<U>::max());
    }

    template <typename T>
    static bool write_enum(uint8_t *output, T value)
    {
        using U = typename std::underlying_type<T>::type;
        static_assert(sizeof(U) <= 4, "enum wire supports at most 32-bit underlying types");
        const int64_t number = static_cast<U>(value);
        if (number < INT32_MIN || number > INT32_MAX) {
            return false;
        }
        write(output, uint32_t(number), 4);
        return true;
    }

    template <typename T>
    static bool in_range(T value, T low, T high)
    {
        return value >= low && value <= high;
    }

    // Parameter names/prefixes occupy a fixed, canonically zero-padded field.
    static const char *read_string(const uint8_t *p)
    {
        uint8_t length = 0;
        while (length < 17 && p[length] != 0) {
            length++;
        }
        if (length == 0 || length == 17) {
            return nullptr;
        }
        for (uint8_t i = length; i < 17; i++) {
            if (p[i] != 0) {
                return nullptr;
            }
        }
        return reinterpret_cast<const char *>(p);
    }
    // Borrowed call-duration string; empty is valid, unused bytes are zero.
    static const char *read_cstring(const uint8_t *p)
    {
        uint16_t length = 0;
        while (length < 256 && p[length] != 0) {
            length++;
        }
        if (length == 256) {
            return nullptr;
        }
        for (uint16_t i = length; i < 256; i++) {
            if (p[i] != 0) {
                return nullptr;
            }
        }
        return reinterpret_cast<const char *>(p);
    }
    static uint32_t read(const uint8_t *p, uint8_t size)
    {
        uint32_t value = 0;
        for (uint8_t i = 0; i < size; i++) {
            value |= uint32_t(p[i]) << (8U * i);
        }
        return value;
    }
    static float read_float(const uint8_t *p)
    {
        const uint32_t bits = read(p, 4);
        float value;
        static_assert(sizeof(value) == sizeof(bits), "32-bit floats required");
        memcpy(&value, &bits, sizeof(value));
        return value;
    }
    static void write(uint8_t *p, uint32_t value, uint8_t size)
    {
        for (uint8_t i = 0; i < size; i++) {
            p[i] = uint8_t(value >> (8U * i));
        }
    }
    static void write_float(uint8_t *p, float value)
    {
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        write(p, bits, 4);
    }

    // Only generated bindings for layout-audited plain float values use this.
    // All current firmware targets are little-endian; fail rather than silently
    // changing the public wire format if that ever changes.
    template <typename T>
    static __attribute__((noinline)) void write_native(uint8_t *p, const T &value)
    {
        static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "little-endian binding wire required");
        memcpy(p, &value, sizeof(value));
    }
};

}
