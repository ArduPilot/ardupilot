// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_HAL/AP_HAL.h>

#if AP_MODULE_ENABLED
#include <AP_HAL/Util.h>
#include <string.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#endif

// Keep compiler/architecture-specific va_list construction in the HAL, never
// in modules or their wire protocol. The layouts below are ABI-specific.
#if (defined(__arm__) && defined(__ARM_EABI__)) || (defined(__x86_64__) && !defined(_WIN32)) || \
    (defined(__aarch64__) && !defined(_WIN32))
namespace
{
using Arg = AP_HAL::Util::FormatArg;

bool decimal(const char *&p, unsigned limit)
{
    unsigned value = 0;
    while (*p >= '0' && *p <= '9') {
        value = value * 10 + unsigned(*p++ - '0');
        if (value > limit) {
            return false;
        }
    }
    return true;
}

template<typename T>
void append(uint8_t *buffer, size_t &offset, const T &value)
{
    offset = (offset + alignof(T) - 1) & ~(alignof(T) - 1);
#if defined(__aarch64__) && defined(__AARCH64EB__)
    // AAPCS64 right-justifies scalars smaller than eight bytes on big-endian.
    memcpy(buffer + offset + 8 - sizeof(T), &value, sizeof(value));
#else
    memcpy(buffer + offset, &value, sizeof(value));
#endif
#if defined(__x86_64__) || defined(__aarch64__)
    offset += (sizeof(T) + 7) & ~size_t(7);
#else
    offset += sizeof(T);
#endif
}

bool marshal(const char *format, const Arg *arguments, uint8_t count, uint8_t *buffer)
{
    size_t offset = 0;
    uint8_t index = 0;
    for (const char *p = format; *p != 0;) {
        if (*p++ != '%') {
            continue;
        }
        if (*p == '%') {
            p++;
            continue;
        }
        while (*p != 0 && strchr("-+ #0", *p) != nullptr) {
            p++;
        }
        if (!decimal(p, 255)) {
            return false;
        }
        if (*p == '.') {
            p++;
            if (!decimal(p, 60)) {
                return false;
            }
        }
        bool wide = false;
        if (*p == 'l') {
            p++;
            if (*p++ != 'l') {
                return false;
            }
            wide = true;
        }
        if (*p == 0 || index == count) {
            return false;
        }
        const char conversion = *p++;
        const Arg &arg = arguments[index++];
        if (conversion == 'd' || conversion == 'i') {
            if (arg.type != (wide ? Arg::Type::INT64 : Arg::Type::INT32)) {
                return false;
            }
            if (wide) {
                append(buffer, offset, static_cast<long long>(arg.i64));
            } else {
                append(buffer, offset, static_cast<int>(arg.i32));
            }
        } else if (strchr("uoxX", conversion) != nullptr) {
            if (arg.type != (wide ? Arg::Type::UINT64 : Arg::Type::UINT32)) {
                return false;
            }
            if (wide) {
                append(buffer, offset, static_cast<unsigned long long>(arg.u64));
            } else {
                append(buffer, offset, static_cast<unsigned int>(arg.u32));
            }
        } else if (!wide && conversion == 'c' && arg.type == Arg::Type::INT32) {
            if (arg.i32 <= 0 || arg.i32 > 127) {
                return false; // keep formatted text NUL-free and UTF-8
            }
            append(buffer, offset, static_cast<int>(arg.i32));
        } else if (!wide && conversion == 's' && arg.type == Arg::Type::STRING && arg.text != nullptr) {
            append(buffer, offset, arg.text);
#if CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS || __FPU_PRESENT
        } else if (!wide && strchr("fFeEgG", conversion) != nullptr && arg.type == Arg::Type::FLOAT64) {
            append(buffer, offset, arg.real);
#endif
        } else {
            return false;
        }
    }
    return index == count;
}
}
#endif

int AP_HAL::Util::snprintf_args(char *str, size_t size, const char *format,
                                const FormatArg *arguments, uint8_t count)
{
#if (defined(__arm__) && defined(__ARM_EABI__)) || (defined(__x86_64__) && !defined(_WIN32)) || \
    (defined(__aarch64__) && !defined(_WIN32))
    static_assert(sizeof(int) == 4 && sizeof(long long) == 8 && sizeof(double) == 8, "printf argument ABI");
    if (format == nullptr || (size != 0 && str == nullptr) || count > FORMAT_MAX_ARGS ||
        (count != 0 && arguments == nullptr)) {
        return -1;
    }
    // At most eight scalar arguments, each occupying no more than eight bytes.
    alignas(8) uint8_t buffer[FORMAT_MAX_ARGS * 8];
    if (!marshal(format, arguments, count, buffer)) {
        return -1;
    }
    va_list ap;
#if defined(__x86_64__) && !defined(__CYGWIN__)
    // SysV AMD64: exhaust GP/XMM register slots so va_arg uses only the
    // eight-byte stack slots in overflow_arg_area. No register save area needed.
    struct Layout {
        unsigned gp_offset;
        unsigned fp_offset;
        void *overflow_arg_area;
        void *reg_save_area;
    } layout {48, 176, buffer, nullptr};
    static_assert(sizeof(ap) == sizeof(layout), "SysV AMD64 va_list layout");
    memcpy(&ap, &layout, sizeof(ap));
#elif defined(__APPLE__) && defined(__aarch64__)
    // Darwin ARM64 passes the supported variadic scalars in eight-byte stack
    // slots, unlike generic AAPCS64. va_list points to the next stack slot.
    void *cursor = buffer;
    static_assert(sizeof(ap) == sizeof(cursor), "Darwin ARM64 va_list layout");
    memcpy(&ap, &cursor, sizeof(ap));
#elif defined(__aarch64__)
    // AAPCS64: exhaust both register save areas so va_arg reads all supported
    // scalars from consecutive eight-byte stack slots, including doubles.
    struct Layout {
        void *stack;
        void *gr_top;
        void *vr_top;
        int gr_offs;
        int vr_offs;
    } layout {buffer, nullptr, nullptr, 0, 0};
    static_assert(sizeof(ap) == sizeof(layout), "AAPCS64 va_list layout");
    memcpy(&ap, &layout, sizeof(ap));
#elif defined(__CYGWIN__)
    // Windows x64 va_list is a pointer into consecutive eight-byte slots,
    // including slots for promoted integers, pointers and double values.
    // Cygwin follows this ABI even though it does not define _WIN32.
    void *cursor = buffer;
    static_assert(sizeof(ap) == sizeof(cursor), "Windows x64 va_list layout");
    memcpy(&ap, &cursor, sizeof(ap));
#else
    // AAPCS32 __va_list contains one pointer; va_arg aligns 64-bit scalars to
    // eight bytes even with the hard-float ABI (variadic floats use base PCS).
    void *cursor = buffer;
    static_assert(sizeof(ap) == sizeof(cursor), "AAPCS32 va_list layout");
    memcpy(&ap, &cursor, sizeof(ap));
#endif
    const int length = vsnprintf(str, size, format, ap);
    va_end(ap);
    return length;
#else
#error "Unsupported architecture for module support"
#endif
}
#endif // AP_MODULE_ENABLED
