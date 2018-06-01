#pragma once

#include <stdint.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    FIRMWARE_VERSION_TYPE fw_type;
    const char *fw_string;
    const char *fw_hash_str;
    const char *middleware_hash_str;
    const char *os_hash_str;
} AP_FWVersion;
