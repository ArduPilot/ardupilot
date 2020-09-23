#pragma once

#include <stdint.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_FWVersion {

public:

    const uint8_t major;
    const uint8_t minor;
    const uint8_t patch;
    const FIRMWARE_VERSION_TYPE fw_type;
    const char *fw_string;
    const char *fw_hash_str;
    const char *middleware_name;
    const char *middleware_hash_str;
    const char *os_name;
    const char *os_hash_str;
    const uint32_t os_sw_version;

    static const AP_FWVersion &get_fwverz() { return fwver; }

private:

    static const AP_FWVersion fwver;
};

namespace AP {
    const AP_FWVersion &fwversion();
};
