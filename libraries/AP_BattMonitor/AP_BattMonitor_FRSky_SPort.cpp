#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_FRSky_SPort.h"
#include "../AP_Frsky_Sensor/AP_Frsky_Sensor.h"

extern const AP_HAL::HAL& hal;
AP_Frsky_Sensor *frsky_sensor;

/// Constructor
AP_BattMonitor_FRSky_SPort::AP_BattMonitor_FRSky_SPort(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _state.healthy = true;
    _id = params._serial_number;
    frsky_sensor = AP_Frsky_Sensor::get_singleton();
}

void AP_BattMonitor_FRSky_SPort::init()
{

}

// read - read the voltage and current
void AP_BattMonitor_FRSky_SPort::read()
{
    uint32_t now = AP_HAL::millis();

    // get voltage reading from the frsky sensor library
    _state.voltage = frsky_sensor->get_SPort_voltage(_id);

    // get the cell data readings from the frsky sensor library
    uint8_t frsky_num_cells = frsky_sensor->get_SPort_num_cells(_id);
    for(uint8_t i=0;i < frsky_num_cells;i++) {
        _state.cell_voltages.cells[i] = frsky_sensor->get_SPort_cells(_id, i);
    }

    // get the last time data was read successfully from the sensor
    // If no contact with the sensor for more than a second then set values to 0.
    if((now - frsky_sensor->get_SPort_read_time(_id)) > 1000 ) {
        _state.voltage = 0;
        for(uint8_t i=0;i < FRSKY_FLVSS_MAX_CELLS;i++) {
            _state.cell_voltages.cells[i] = 0;
        }
    }
}

/// return true if battery provides current info
bool AP_BattMonitor_FRSky_SPort::has_current() const
{
    return false;
}

bool AP_BattMonitor_FRSky_SPort::has_cell_voltages()
{
    if(_state.cell_voltages.cells[0] > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
