#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_ESC_Telem {
public:

    AP_ESC_Telem();

    /* Do not allow copies */
    AP_ESC_Telem(const AP_ESC_Telem &other) = delete;
    AP_ESC_Telem &operator=(const AP_ESC_Telem&) = delete;

    static AP_ESC_Telem *get_singleton();

    // get an individual ESC's usage time in seconds if available, returns true on success
    bool get_usage_seconds(uint8_t esc_id, uint32_t& usage_sec) const;

private:

    static AP_ESC_Telem *_singleton;

};

namespace AP {
    AP_ESC_Telem &esc_telem();
};
