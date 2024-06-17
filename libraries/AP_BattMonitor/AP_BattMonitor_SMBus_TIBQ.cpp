#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SMBUS_TIBQ_ENABLED

#include "AP_BattMonitor_SMBus_TIBQ.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>


// Extention of AP_BattMonitor_SMBus_Generic to include TI's BQ40Z chip shutdown mechanism
AP_BattMonitor_SMBus_TIBQ::AP_BattMonitor_SMBus_TIBQ(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params)
    : AP_BattMonitor_SMBus_Generic(mon, mon_state, params)
{
    _exit_emshut = true;
}


void AP_BattMonitor_SMBus_TIBQ::timer() {
    if (_exit_emshut) {
        // Exit EMERGENCY SHUTDOWN state in case it was engaged on the last poweroff:
        uint8_t cmd[] = {0x00, 0xA7, 0x23};
        if (_dev->transfer(cmd, 3, nullptr, 0)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BQ40Z bms exited shutdown");
            _exit_emshut = false;
        }
    }

    AP_BattMonitor_SMBus_Generic::timer();
}


bool AP_BattMonitor_SMBus_TIBQ::shutdown() {
    // Semaphore is needed in case this is called from another thread
    WITH_SEMAPHORE(_dev->get_semaphore());

    uint8_t cmd[] = {0x00, 0x10, 0x00};
    if (!_dev->transfer(cmd, 3, nullptr, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to shutdown TIBQ");
        return false;
    } else {
        _state.is_powering_off = true;
    }

    return true;
}

#endif // AP_BATTERY_SMBUS_TIBQ_ENABLED
