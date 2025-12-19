#include "AP_LeakDetector.h"
#include "AP_LeakDetector_Analog.h"
#include "AP_LeakDetector_Digital.h"

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_LeakDetector::var_info[] = {

    // @Param: 1_PIN
    // @DisplayName: Pin that leak detector is connected to
    // @Description: Pin that the leak detector is connected to
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,13:Pixhawk 3.3ADC1,14:Pixhawk 3.3ADC2,15:Pixhawk 6.6ADC,27:Navigator Built-In
    // @Range: -1 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("1_PIN", 1, AP_LeakDetector, _pin[0], -1),

    // @Param: 1_LOGIC
    // @DisplayName: Default reading of leak detector when dry
    // @Description: Default reading of leak detector when dry
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("1_LOGIC", 2, AP_LeakDetector, _default_reading[0], 0),

    // 7 was 1_TYPE, specifying analog or digital input type

#if LEAKDETECTOR_MAX_INSTANCES > 1
    // @Param: 2_PIN
    // @DisplayName: Pin that leak detector is connected to
    // @Description: Pin that the leak detector is connected to
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,13:Pixhawk 3.3ADC1,14:Pixhawk 3.3ADC2,15:Pixhawk 6.6ADC,27:Navigator Leak1
    // @Range: -1 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("2_PIN", 3, AP_LeakDetector, _pin[1], -1),

    // @Param: 2_LOGIC
    // @DisplayName: Default reading of leak detector when dry
    // @Description: Default reading of leak detector when dry
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("2_LOGIC", 4, AP_LeakDetector, _default_reading[1], 0),

    // 8 was 2_TYPE, specifying analog or digital input type
#endif

#if LEAKDETECTOR_MAX_INSTANCES > 2
    // @Param: 3_PIN
    // @DisplayName: Pin that leak detector is connected to
    // @Description: Pin that the leak detector is connected to
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,13:Pixhawk 3.3ADC1,14:Pixhawk 3.3ADC2,15:Pixhawk 6.6ADC,27:Navigator Leak1
    // @Range: -1 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("3_PIN", 5, AP_LeakDetector, _pin[2], -1),

    // @Param: 3_LOGIC
    // @DisplayName: Default reading of leak detector when dry
    // @Description: Default reading of leak detector when dry
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("3_LOGIC", 6, AP_LeakDetector, _default_reading[2], 0),

    // 9 was 3_TYPE, specifying analog or digital input type
#endif

    AP_GROUPEND
};

AP_LeakDetector::AP_LeakDetector() :
    _status(false),
    _last_detect_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    memset(_state,0,sizeof(_state));
    memset(_drivers,0,sizeof(_drivers));
};

void AP_LeakDetector::init()
{
    for (int i = 0; i < LEAKDETECTOR_MAX_INSTANCES; i++) {
        if (_pin[i] < 0) {
            continue;
        }
        if (hal.analogin->valid_analog_pin(_pin[i])) {
                _state[i].instance = i;
                _drivers[i] = NEW_NOTHROW AP_LeakDetector_Analog(*this, _state[i]);
        } else {
                _state[i].instance = i;
                _drivers[i] = NEW_NOTHROW AP_LeakDetector_Digital(*this, _state[i]);
        }
    }
}

bool AP_LeakDetector::update()
{
    uint32_t tnow = AP_HAL::millis();

    for (int i = 0; i < LEAKDETECTOR_MAX_INSTANCES; i++) {
        if (_drivers[i] != NULL) {
            _drivers[i]->read();
            if (_state[i].status) {
                _last_detect_ms = tnow;
            }
        }
    }

    _status = tnow < _last_detect_ms + LEAKDETECTOR_COOLDOWN_MS;

    return _status;
}

void AP_LeakDetector::set_detect()
{
    _last_detect_ms = AP_HAL::millis();
}

int8_t AP_LeakDetector::get_pin(uint8_t instance) const
{
    if (instance >= LEAKDETECTOR_MAX_INSTANCES) {
        return 0;
    }
    return _pin[instance];
}
