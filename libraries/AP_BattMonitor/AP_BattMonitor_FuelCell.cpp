#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_FuelCell.h"
#include "AP_FuelCell/AP_FuelCell.h"

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_FuelCell::AP_BattMonitor_FuelCell(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _state.voltage = 1.0; // show a fixed voltage of 1v
}

// read - read the voltage and current
void AP_BattMonitor_FuelCell::read()
{
#if ENABLE_FUELCELL
    // get fuel cell lib pointer
    const AP_FuelCell* fuel_cell = AP_FuelCell::get_singleton();
    if (fuel_cell == nullptr) {
        _state.healthy = false;
        return;
    }
    // check that it is enabled
    if (!fuel_cell->enabled()) {
        _state.healthy = false;
        return;
    }
    _state.healthy = fuel_cell->healthy();

    float proportion_remaining = 0.0f;
    switch (_params._type) {
        case AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_TANK:
            proportion_remaining = fuel_cell->get_tank();
            _state.last_time_micros = AP_HAL::micros();

            break;
        case AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_BATTERY:
            proportion_remaining = fuel_cell->get_battery();
            _state.last_time_micros = AP_HAL::micros();
            break;
        default:
            _state.healthy = false;
            break;
    }

    // map consumed_mah to consumed percentage
    _state.consumed_mah = (1 - proportion_remaining) * _params._pack_capacity;

    // map consumed_wh using fixed voltage of 1
    _state.consumed_wh = _state.consumed_mah;
#else 
    _state.healthy = false;
#endif
}

AP_BattMonitor::BatteryFailsafe AP_BattMonitor_FuelCell::update_failsafes()
{
#if ENABLE_FUELCELL

    AP_BattMonitor::BatteryFailsafe fuel_cell_failsafe = AP_BattMonitor::BatteryFailsafe::BatteryFailsafe_None;

    // only check for fuel cell failsafes on the tank moniter
    // no point in having the same failsafe on two battery's
    if (_params._type == AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_TANK) {
        // get fuel cell lib pointer
        const AP_FuelCell* fuel_cell = AP_FuelCell::get_singleton();
        if (fuel_cell != nullptr) {
            fuel_cell_failsafe = fuel_cell->update_failsafes();
        }
    }

    return MAX(AP_BattMonitor_Backend::update_failsafes(),fuel_cell_failsafe);
#else
    return AP_BattMonitor_Backend::update_failsafes();
#endif
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
bool AP_BattMonitor_FuelCell::arming_checks(char * buffer, size_t buflen) const
{

    // check standard things
    if (!AP_BattMonitor_Backend::arming_checks(buffer, buflen)) {
        return false;
    }

#if ENABLE_FUELCELL
    // if they pass also check fuel cell lib for arming
    const AP_FuelCell* fuel_cell = AP_FuelCell::get_singleton();
    if (fuel_cell == nullptr) {
        char message[17];
        strcpy(message, "Fuel Cell NulPtr");
        strncpy(buffer, message, buflen);
        return false;
    }

    // only arming check the tank moniter
    if (_params._type == AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_TANK) {
        return fuel_cell->arming_checks(buffer, buflen);
    }
#endif

    // if we got this far we are safe to arm
    return true;
}
