/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"
#include "UserVariables.h"

#ifdef USERHOOK_INIT



#define I2C_ADDRESS_PMS3003  0x08

extern const AP_HAL::HAL& hal;

PMS3003 pms3003;

//============================================================================
// Userhook_init
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    pms3003.init();
    Log_Write_Enviro(pms3003._PM10,
            pms3003._PM25,
            pms3003._PM100,
            pms3003._CP030,
            pms3003._CP050,
            pms3003._CP10,
            pms3003._CP25,
            pms3003._CP50,
            pms3003._CP100,
            pms3003._HUM,
            pms3003._TEMP_C);
}
#endif

//============================================================================
// Userhook_SuperSlowLoop
#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    hal.console->printf("\n\n test userhook loop console \n");
    pms3003._timer();
    Log_Write_Enviro(pms3003._PM10,
            pms3003._PM25,
            pms3003._PM100,
            pms3003._CP030,
            pms3003._CP050,
            pms3003._CP10,
            pms3003._CP25,
            pms3003._CP50,
            pms3003._CP100,
            pms3003._HUM,
            pms3003._TEMP_C);
}


//===========================================================================
// probe and initialize the sensor
bool PMS3003::init(void)
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus semaphore
    if (!i2c_sem->take(200))
        return false;

    _measure();     // measure time
    hal.scheduler->delay(10);
    _collect();     // measure data
    i2c_sem->give();
    if (_last_sample_time_ms != 0) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&PMS3003::_timer, void));
        return true;
    }
    return false;
}
//=========================================================================
// start a measurement of time
void PMS3003::_measure(void)
{
    _measurement_started_ms = 0;
    if (hal.i2c->writeRegisters(I2C_ADDRESS_PMS3003, 0, 0, NULL) == 0) {
        _measurement_started_ms = AP_HAL::millis();
    }
}
//=========================================================================
// read the values from the sensor
void PMS3003::_collect(void)
{
    uint8_t data[20];

    _measurement_started_ms = 0;

    if (hal.i2c->read(I2C_ADDRESS_PMS3003, 20, data) != 0) {
        return;
    }

    _PM10 = (data[0] << 8) + data[1];
    _PM25 = (data[2] << 8) + data[3];
    _PM100 = (data[4] << 8) + data[5];
    _CP030 = (data[6] << 8) + data[7];
    _CP050 = (data[8] << 8) + data[9];
    _CP10 = (data[10] << 8) + data[11];
    _CP25 = (data[12] << 8) + data[13];
    _CP50 = (data[14] << 8) + data[15];
    _CP100 = (data[16] << 8) + data[17];
    _HUM = data[18];
    _TEMP_C = data[19];

    _last_sample_time_ms = AP_HAL::millis();
}
//=========================================================================
// 1kHz timer
void PMS3003::_timer(void)
{
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    if (!i2c_sem->take_nonblocking())
        return;

    if (_measurement_started_ms == 0) {
        _measure();
        i2c_sem->give();
        return;
    }
    if ((AP_HAL::millis() - _measurement_started_ms) > 10) {
        _collect();
        // start a new measurement
        _measure();
    }
    i2c_sem->give();
}

#endif

//============================================================================
// Log.cpp
