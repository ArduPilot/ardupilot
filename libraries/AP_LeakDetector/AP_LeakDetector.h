#pragma once

#include <AP_Param/AP_Param.h>

#define LEAKDETECTOR_MAX_INSTANCES 3

#define LEAKDETECTOR_COOLDOWN_MS 3000 // Status will return true for this long after last time leak was detected

class AP_LeakDetector_Backend;

class AP_LeakDetector {

    friend class AP_LeakDetector_Analog;
    friend class AP_LeakDetector_Digital;

public:
    AP_LeakDetector();

    enum LeakDetector_Type {
        LEAKDETECTOR_TYPE_NONE = 0,
        LEAKDETECTOR_TYPE_DIGITAL = 1,
        LEAKDETECTOR_TYPE_ANALOG = 2
    };

    struct LeakDetector_State {
        uint8_t instance;
        bool status; // leaking?
    };

    // Return current status
    bool get_status(void) const { return _status; }

    // Set status externally, ie via mavlink message from subsystems
    void set_detect(void);

    // Initialize all drivers
    void init(void);

    // Update all drivers
    bool update(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_LeakDetector_Backend *_drivers[LEAKDETECTOR_MAX_INSTANCES];
    LeakDetector_State _state[LEAKDETECTOR_MAX_INSTANCES];

    bool _status; // Current status, true if leak detected, false if all sensors dry
    uint32_t _last_detect_ms;

    AP_Int8 _type[LEAKDETECTOR_MAX_INSTANCES]; // Analog, Digital, Mavlink
    AP_Int8 _pin[LEAKDETECTOR_MAX_INSTANCES]; // Pin that detector is connected to
    AP_Int8 _default_reading[LEAKDETECTOR_MAX_INSTANCES]; // Default reading when leak detector is dry
};
