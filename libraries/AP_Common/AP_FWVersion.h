#pragma once

#include <stdint.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class PACKED AP_FWVersion {

public:
    /**
     * @brief Struct to hold infomation about the software version struct
     *
     */
    // First 7 MSBs are a start sequence, LSB is a checksum
    const uint64_t header;
    // MSB (major version breaks compatibility), LSB (minor version no compatibility break)
    const uint16_t header_version;
    // Pointer size to extract pointer values
    const uint8_t pointer_size;

    const uint8_t reserved; // padding
    const uint8_t vehicle_type;
    const uint8_t board_type;
    const uint16_t board_subtype;
    const uint8_t major;
    const uint8_t minor;
    const uint8_t patch;
    const uint8_t fw_type; /*FIRMWARE_VERSION_TYPE*/
    const uint32_t os_sw_version;
    const char *fw_string;
    const char *fw_hash_str;
    const char *fw_short_string;
    const char *middleware_name;
    const char *middleware_hash_str;
    const char *os_name;
    const char *os_hash_str;

    static const AP_FWVersion &get_fwverz() { return fwver; }

private:

    static const AP_FWVersion fwver;
};

namespace AP {
    const AP_FWVersion &fwversion();
};
