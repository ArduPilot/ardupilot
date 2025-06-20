#include "AP_LeakDetector.h"
#include "AP_LeakDetector_Analog.h"
#include "AP_LeakDetector_Digital.h"

const AP_Param::GroupInfo AP_LeakDetector::var_info[] = {

    // @Param: 1_PIN
    // @DisplayName: Pin that leak detector is connected to
    // @Description: Pin that the leak detector is connected to
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,13:Pixhawk 3.3ADC1,14:Pixhawk 3.3ADC2,15:Pixhawk 6.6ADC,27:Navigator Built-In
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("1_PIN", 1, AP_LeakDetector, _pin[0], -1),

    // @Param: 1_LOGIC
    // @DisplayName: Default reading of leak detector when dry
    // @Description: Default reading of leak detector when dry
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("1_LOGIC", 2, AP_LeakDetector, _default_reading[0], 0),

    // @Param: 1_TYPE
    // @DisplayName: Leak detector pin type (analog/digital)
    // @Description: Enables leak detector 1. Use this parameter to indicate the signal type (0:analog, 1:digital) of an appropriately configured input pin, then specify its pin number using the LEAK1_PIN parameter. NOT FOR USE by default with Pixhawk, Pixhawk 4 or Navigator flight controllers.
    // @Values: -1:Disabled,0:Analog,1:Digital
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("1_TYPE", 7, AP_LeakDetector, _type[0], DISABLED), 

#if LEAKDETECTOR_MAX_INSTANCES > 1
    // @Param: 2_PIN
    // @DisplayName: Pin that leak detector is connected to
    // @Description: Pin that the leak detector is connected to
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,13:Pixhawk 3.3ADC1,14:Pixhawk 3.3ADC2,15:Pixhawk 6.6ADC,27:Navigator Leak1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("2_PIN", 3, AP_LeakDetector, _pin[1], -1),

    // @Param: 2_LOGIC
    // @DisplayName: Default reading of leak detector when dry
    // @Description: Default reading of leak detector when dry
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("2_LOGIC", 4, AP_LeakDetector, _default_reading[1], 0),

    // @Param: 2_TYPE
    // @DisplayName: Leak detector pin type (analog/digital)
    // @Description: Enables leak detector 2. Use this parameter to indicate the signal type (0:analog, 1:digital) of an appropriately configured input pin, then specify its pin number using the LEAK2_PIN parameter. NOT FOR USE by default with Pixhawk, Pixhawk 4 or Navigator flight controllers.
    // @Values: -1:Disabled,0:Analog,1:Digital
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("2_TYPE", 8, AP_LeakDetector, _type[1], DISABLED),
#endif

#if LEAKDETECTOR_MAX_INSTANCES > 2
    // @Param: 3_PIN
    // @DisplayName: Pin that leak detector is connected to
    // @Description: Pin that the leak detector is connected to
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,13:Pixhawk 3.3ADC1,14:Pixhawk 3.3ADC2,15:Pixhawk 6.6ADC,27:Navigator Leak1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("3_PIN", 5, AP_LeakDetector, _pin[2], -1),

    // @Param: 3_LOGIC
    // @DisplayName: Default reading of leak detector when dry
    // @Description: Default reading of leak detector when dry
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("3_LOGIC", 6, AP_LeakDetector, _default_reading[2], 0),

    // @Param: 3_TYPE
    // @DisplayName: Leak detector pin type (analog/digital)
    // @Description: Enables leak detector 3. Use this parameter to indicate the signal type (0:analog, 1:digital) of an appropriately configured input pin, then specify its pin number using the LEAK3_PIN parameter. NOT FOR USE by default with Pixhawk, Pixhawk 4 or Navigator flight controllers.
    // @Values: -1:Disabled,0:Analog,1:Digital
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("3_TYPE", 9, AP_LeakDetector, _type[2], DISABLED),
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

AP_LeakDetector::_signal_types AP_LeakDetector::get_default_signal_type_for_pin(uint8_t pin) const
{
#ifdef AP_LEAKDETECTOR_PIN_IS_ANALOG
    static const uint8_t analog_pins[] { AP_LEAKDETECTOR_PIN_IS_ANALOG };
    for (auto apin : analog_pins) {
        if (apin == pin) {
            return _signal_types::ANALOG;
        }
    }
#endif  // AP_LEAKDETECTOR_PIN_IS_ANALOG
#ifdef AP_LEAKDETECTOR_PIN_IS_DIGITAL
    static const uint8_t digital_pins[] { AP_LEAKDETECTOR_PIN_IS_DIGITAL };
    for (auto dpin : digital_pins) {
        if (dpin == pin) {
            return _signal_types::DIGITAL;
        }
    }
#endif  // AP_LEAKDETECTOR_PIN_IS_DIGITAL
    return _signal_types::DISABLED;
}

void AP_LeakDetector::init()
{
    for (int i = 0; i < LEAKDETECTOR_MAX_INSTANCES; i++) {
        const _signal_types t = get_default_signal_type_for_pin(_pin[i]);
        if (t != _signal_types::DISABLED) {
            _type[i].set_default(t);
        }

        switch(_type[i]) {
            case ANALOG:
                _state[i].instance = i;
                _drivers[i] = NEW_NOTHROW AP_LeakDetector_Analog(*this, _state[i]);
                break;
            case DIGITAL:
                _state[i].instance = i;
                _drivers[i] = NEW_NOTHROW AP_LeakDetector_Digital(*this, _state[i]);
                break;
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
