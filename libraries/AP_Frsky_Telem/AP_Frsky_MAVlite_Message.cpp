#include "AP_Frsky_MAVlite_Message.h"

#include <AP_Math/AP_Math.h>

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
bool AP_Frsky_MAVlite_Message::get_bytes(uint8_t *bytes, const uint8_t offset, const uint8_t count) const
{
    if (offset + count > MAVLITE_MAX_PAYLOAD_LEN) {
        return false;
    }
    memcpy(bytes, &payload[offset], count);
    return true;
}

bool AP_Frsky_MAVlite_Message::set_bytes(const uint8_t *bytes, const uint8_t offset, const uint8_t count)
{
    if (offset + count > MAVLITE_MAX_PAYLOAD_LEN) {
        return false;
    }
    memcpy(&payload[offset], bytes, count);
    len += count;
    return true;
}

bool AP_Frsky_MAVlite_Message::get_string(char* value, const uint8_t offset) const
{
    if (get_bytes((uint8_t*)value, offset, MIN((uint8_t)16, len - offset))) {
        value[MIN((uint8_t)16, len - offset)] = 0x00; // terminator
        return true;
    }
    return false;
}

bool AP_Frsky_MAVlite_Message::set_string(const char* value, const uint8_t offset)
{
    return set_bytes((uint8_t*)value, offset, MIN((uint8_t)16, strlen(value)));
}


uint8_t AP_Frsky_MAVlite_Message::bit8_unpack(const uint8_t value, const uint8_t bit_count, const uint8_t bit_offset)
{
    uint8_t mask = 0;
    for (uint8_t i=bit_offset; i<=bit_count; i++) {
        mask |= 1 << i;
    }
    return (value & mask) >> bit_offset;
}

void AP_Frsky_MAVlite_Message::bit8_pack(uint8_t &value, const uint8_t bit_value, const uint8_t bit_count, const uint8_t bit_offset)
{
    uint8_t mask = 0;
    for (uint8_t i=bit_offset; i<=bit_count; i++) {
        mask |= 1 << i;
    }
    value |= (bit_value<<bit_offset) & mask;
}
#endif