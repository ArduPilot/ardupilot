#include "AP_LeakDetector.h"
#include "AP_LeakDetector_Analog.h"
#include "AP_LeakDetector_Digital.h"

const AP_Param::GroupInfo AP_LeakDetector::var_info[] = {

    // @Param: 1_PIN
    // @DisplayName: Pin that leak detector is connected to
    // @Description: Pin that the leak detector is connected to
    // @Values: -1:Disabled,50:Pixhawk Aux1,51:Pixhawk Aux2,52:Pixhawk Aux3,53:Pixhawk Aux4,54:Pixhawk Aux5,55:Pixhawk Aux6,13:Pixhawk 3.3ADC1,14:Pixhawk 3.3ADC2,15:Pixhawk 6.6ADC
    // @User: Standard
    AP_GROUPINFO("1_PIN", 1, AP_LeakDetector, _pin[0], -1),

    // @Param: 1_LOGIC
    // @DisplayName: Default reading of leak detector when dry
    // @Description: Default reading of leak detector when dry
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("1_LOGIC", 2, AP_LeakDetector, _default_reading[0], 0),

#if LEAKDETECTOR_MAX_INSTANCES > 1
    // @Param: 2_PIN
    // @DisplayName: Pin that leak detector is connected to
    // @Description: Pin that the leak detector is connected to
    // @Values: -1:Disabled,50:Pixhawk Aux1,51:Pixhawk Aux2,52:Pixhawk Aux3,53:Pixhawk Aux4,54:Pixhawk Aux5,55:Pixhawk Aux6,13:Pixhawk 3.3ADC1,14:Pixhawk 3.3ADC2,15:Pixhawk 6.6ADC
    // @User: Standard
    AP_GROUPINFO("2_PIN", 3, AP_LeakDetector, _pin[1], -1),

    // @Param: 2_LOGIC
    // @DisplayName: Default reading of leak detector when dry
    // @Description: Default reading of leak detector when dry
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("2_LOGIC", 4, AP_LeakDetector, _default_reading[1], 0),
#endif

#if LEAKDETECTOR_MAX_INSTANCES > 2
    // @Param: 3_PIN
    // @DisplayName: Pin that leak detector is connected to
    // @Description: Pin that the leak detector is connected to
    // @Values: -1:Disabled,50:Pixhawk Aux1,51:Pixhawk Aux2,52:Pixhawk Aux3,53:Pixhawk Aux4,54:Pixhawk Aux5,55:Pixhawk Aux6,13:Pixhawk 3.3ADC1,14:Pixhawk 3.3ADC2,15:Pixhawk 6.6ADC
    // @User: Standard
    AP_GROUPINFO("3_PIN", 5, AP_LeakDetector, _pin[2], -1),

    // @Param: 3_LOGIC
    // @DisplayName: Default reading of leak detector when dry
    // @Description: Default reading of leak detector when dry
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("3_LOGIC", 6, AP_LeakDetector, _default_reading[2], 0),
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
        switch (_pin[i]) {
        case 50 ... 55:
            _state[i].instance = i;
            _drivers[i] = new AP_LeakDetector_Digital(*this, _state[i]);
            break;
        case 13 ... 15:
            _state[i].instance = i;
            _drivers[i] = new AP_LeakDetector_Analog(*this, _state[i]);
            break;
        default:
            _drivers[i] = NULL;
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
