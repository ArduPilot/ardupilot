// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingLog.h"

#if AP_MODULE_ENABLED
#include <AP_Module/BindingLog_generated.h>
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

namespace AP_Module
{

#if HAL_LOGGING_ENABLED
static const char *read_metadata(const uint8_t *input, uint8_t size)
{
    uint8_t end = 0;
    while (end < size && input[end] != 0) {
        end++;
    }
    if (end == size) {
        return nullptr;
    }
    for (uint8_t i = end; i < size; i++) {
        if (input[i] != 0) {
            return nullptr;
        }
    }
    return reinterpret_cast<const char *>(input);
}

static bool valid_labels(const char *labels, uint8_t fields)
{
    const char *start[16];
    uint8_t length[16];
    uint8_t count = 0;
    const char *part = labels;
    for (const char *p = labels; ; p++) {
        if (*p != ',' && *p != 0) {
            continue;
        }
        if (p == part || count >= 16) {
            return false;
        }
        const uint8_t n = p - part;
        for (uint8_t i = 0; i < count; i++) {
            if (length[i] == n && memcmp(start[i], part, n) == 0) {
                return false;
            }
        }
        start[count] = part;
        length[count++] = n;
        if (*p == 0) {
            return count == fields;
        }
        part = p + 1;
    }
}

template <typename T, size_t N>
static bool valid_ids(const char *ids, const T (&table)[N])
{
    for (const char *p = ids; *p != 0; p++) {
        bool found = false;
        for (const auto &entry : table) {
            if (*p == entry.ID) {
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

static bool same_optional(const char *a, const char *b)
{
    return (a == nullptr || b == nullptr) ? a == b : strcmp(a, b) == 0;
}
#endif

int32_t binding_log_write(const uint8_t *input, uint32_t input_size,
                          uint8_t *, uint32_t output_size)
{
    if (input == nullptr || input_size != LOG_WIRE_SIZE || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_LOGGING_ENABLED
    auto *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    const char *name = read_metadata(input, 5);
    const char *labels = read_metadata(input + 5, 58);
    const char *format = read_metadata(input + 63, 16);
    const char *units = read_metadata(input + 79, 16);
    const char *multipliers = read_metadata(input + 95, 16);
    if (name == nullptr || labels == nullptr || format == nullptr || units == nullptr || multipliers == nullptr ||
        name[0] == 0 || labels[0] == 0 || format[0] == 0 || input[111] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    const uint8_t fields = strlen(format);
    const bool have_units = input[111] != 0;
    const uint16_t length = BindingWire::read(input + 112, 2);
    if (length > LOG_WIRE_PAYLOAD_MAX ||
        (have_units && (strlen(units) != fields || strlen(multipliers) != fields)) ||
        (!have_units && (units[0] != 0 || multipliers[0] != 0))) {
        return AP_MODULE_STATUS_INVALID;
    }
    uint16_t expected = 0;
    for (uint8_t i = 0; i < fields; i++) {
        const uint8_t n = log_field_size(format[i]);
        if (n == 0) {
            return AP_MODULE_STATUS_INVALID;
        }
        expected += n;
    }
    if (expected != length) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint16_t i = length; i < LOG_WIRE_PAYLOAD_MAX; i++) {
        if (input[114 + i] != 0) {
            return AP_MODULE_STATUS_INVALID;
        }
    }

    char full_labels[65] = "TimeUS,";
    char full_format[17] = "Q";
    char full_units[17] = "s";
    char full_multipliers[17] = "F";
    memcpy(full_labels + 7, labels, strlen(labels) + 1);
    memcpy(full_format + 1, format, fields + 1);
    if (!valid_labels(full_labels, fields + 1) ||
        !valid_ids(units, log_Units) || !valid_ids(multipliers, log_Multipliers)) {
        return AP_MODULE_STATUS_INVALID;
    }
    if (have_units) {
        memcpy(full_units + 1, units, fields + 1);
        memcpy(full_multipliers + 1, multipliers, fields + 1);
    }
    const char *unit_arg = have_units ? full_units : nullptr;
    const char *multiplier_arg = have_units ? full_multipliers : nullptr;
    const int16_t record_length = logger->Write_calc_msg_len(full_format);
    if (record_length != length + 11 || record_length > LOG_PACKET_MAX_LEN) {
        return AP_MODULE_STATUS_INVALID;
    }
    // The registry copies all metadata, as it does for Lua. Entries persist
    // across module exit/reload and consume shared firmware memory/message IDs.
    auto *entry = logger->msg_fmt_for_name(name, full_labels, unit_arg, multiplier_arg, full_format, true, true);
    if (entry == nullptr) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    // Entries are immutable after publication under the logger's own semaphore.
    // Hardware otherwise silently reuses a conflicting format. SITL already
    // raises config_error inside the lookup on a conflict, as with Lua.
    if (entry->msg_len != record_length || strcmp(entry->name, name) != 0 ||
        strcmp(entry->fmt, full_format) != 0 || strcmp(entry->labels, full_labels) != 0 ||
        !same_optional(entry->units, unit_arg) || !same_optional(entry->mults, multiplier_arg)) {
        return AP_MODULE_STATUS_INVALID;
    }
    uint8_t record[255];
    record[0] = HEAD_BYTE1;
    record[1] = HEAD_BYTE2;
    record[2] = entry->msg_type;
    const uint64_t now = AP_HAL::micros64();
    BindingWire::write(record + 3, uint32_t(now), 4);
    BindingWire::write(record + 7, uint32_t(now >> 32), 4);
    memcpy(record + 11, input + 114, length);
    logger->Safe_Write_Emit_FMT(entry);
    logger->WriteBlock(record, record_length);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif // AP_MODULE_ENABLED
