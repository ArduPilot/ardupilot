#include "AP_BattMonitor_IntelAero.h"

#if AP_BATTERY_INTELAERO_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#ifndef HAL_BATTMON_INTELAERO_BUS
#define HAL_BATTMON_INTELAERO_BUS 1
#endif

#define INTELAERO_ADC_ADDR 0x50
#define INTELAERO_ADC_ENABLE_REG 0x00
#define INTELAERO_ADC_CHANNEL_REG 0x03
#define INTELAERO_ADC_NUM_CHANNELS 5
#define INTELAERO_ADC_VOLTAGE_CHANNEL 1
#define INTELAERO_ADC_VOLTS_PER_COUNT (3.3f / 4096)

const AP_Param::GroupInfo AP_BattMonitor_IntelAero::var_info[] = {

    // Param indexes must be between 7 and 9 to avoid conflict with other battery monitor param tables loaded by pointer

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage measured by the FPGA's ADC to the battery's voltage
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 7, AP_BattMonitor_IntelAero, volt_multiplier, 9.0),

    // CHECK/UPDATE INDEX TABLE IN AP_BattMonitor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

AP_BattMonitor_IntelAero::AP_BattMonitor_IntelAero(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_BattMonitor_IntelAero::init(void)
{
    dev = hal.i2c_mgr->get_device_ptr(HAL_BATTMON_INTELAERO_BUS, INTELAERO_ADC_ADDR, 400000, false, 20);
    if (dev == nullptr) {
        return;
    }
    {
        WITH_SEMAPHORE(dev->get_semaphore());
        dev->set_retries(10);
        if (!dev->write_register(INTELAERO_ADC_ENABLE_REG, 0x01)) {
            dev = nullptr;
            return;
        }
        dev->set_retries(2);
    }
    dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_IntelAero::timer, void));
}

void AP_BattMonitor_IntelAero::timer(void)
{
    uint8_t data[INTELAERO_ADC_NUM_CHANNELS * 2];
    if (!dev->read_registers(INTELAERO_ADC_CHANNEL_REG, data, sizeof(data))) {
        return;
    }
    const uint8_t i = INTELAERO_ADC_VOLTAGE_CHANNEL * 2;
    WITH_SEMAPHORE(sem);
    raw_voltage = data[i] | (data[i+1] << 8);
    last_sample_ms = AP_HAL::millis();
}

void AP_BattMonitor_IntelAero::read()
{
    WITH_SEMAPHORE(sem);
    _state.healthy = last_sample_ms != 0 && AP_HAL::millis() - last_sample_ms < 1000;
    if (!_state.healthy) {
        return;
    }
    _state.voltage = raw_voltage * INTELAERO_ADC_VOLTS_PER_COUNT * volt_multiplier;
    _state.last_time_micros = AP_HAL::micros();
}

#endif  // AP_BATTERY_INTELAERO_ENABLED
