/*******************************************
*   Sample sketch that configures an MPU6000
*   and reads back the three axis of accel,
*   temperature, three axis of gyro data
*******************************************/

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_VRBRAIN.h>

/* register #defines */
//#include "MPU6000.h"

// debug only:
//#include <avr/io.h>


const AP_HAL::HAL& hal = AP_HAL_VRBRAIN;

AP_HAL::SPIDeviceDriver* spidev;

/* Asynchronous state: */
static volatile bool            _updated;
static volatile uint8_t         _DD1_count;
static volatile uint8_t         _DD2_count;
static volatile uint32_t        _s_DD1, _s_DD2;
static uint8_t                  _state;
static uint32_t                 _timer;
/* Gates access to asynchronous state: */

float                           Temp;
float                           Press;

int32_t                         _raw_press;
int32_t                         _raw_temp;
// Internal calibration registers
uint16_t                        C1,C2,C3,C4,C5,C6;
float                           DD1,DD2;
uint32_t                            _last_update;
uint8_t                             _pressure_samples;
float                            _ground_temperature;
float                            _ground_pressure;
float                               _altitude;
uint32_t                            _last_altitude_t;

#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_DD1_OSR4096 0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_DD2_OSR4096 0x58   // Maximum resolution (oversampling)


uint16_t read_16bits(uint8_t reg)
{
    uint8_t tx[3];
    uint8_t rx[3];
    tx[0] = reg; tx[1] = 0; tx[2] = 0;
    spidev->transaction(tx, rx, 3);
    return ((uint16_t) rx[1] << 8 ) | ( rx[2] );
}

uint32_t read_adc()
{
    uint8_t tx[4];
    uint8_t rx[4];
    memset(tx, 0, 4); /* first byte is addr = 0 */
    spidev->transaction(tx, rx, 4);
    return (((uint32_t)rx[1])<<16) | (((uint32_t)rx[2])<<8) | ((uint32_t)rx[3]);
}


void write(uint8_t reg)
{
    uint8_t tx[1];
    tx[0] = reg;
    spidev->transaction(tx, NULL, 1);
}

static void ms5611_init(void) {
    // chip reset
    write(CMD_MS5611_RESET);
        hal.scheduler->delay(4);

        // We read the factory calibration
        // The on-chip CRC is not used
        C1 = read_16bits(CMD_MS5611_PROM_C1);
        C2 = read_16bits(CMD_MS5611_PROM_C2);
        C3 = read_16bits(CMD_MS5611_PROM_C3);
        C4 = read_16bits(CMD_MS5611_PROM_C4);
        C5 = read_16bits(CMD_MS5611_PROM_C5);
        C6 = read_16bits(CMD_MS5611_PROM_C6);


        //Send a command to read Temp first
        write(CMD_CONVERT_DD2_OSR4096);
        _timer = hal.scheduler->micros();
        _state = 0;
        Temp=0;
        Press=0;

        _s_DD1 = 0;
        _s_DD2 = 0;
        _DD1_count = 0;
        _DD2_count = 0;

        // wait for at least one value to be read
        uint32_t tstart = hal.scheduler->millis();
        while (!_updated){
            _update(hal.scheduler->micros());
            hal.scheduler->delay(10);
            if (hal.scheduler->millis() - tstart > 1000) {
                hal.scheduler->panic(PSTR("PANIC: AP_Baro_MS5611 took more than "
                            "1000ms to initialize"));
                //healthy = false;
                //return false;
            }
        }

        //healthy = true;
        //return true;

}

void _update(uint32_t tnow)
{
    // Throttle read rate to 100hz maximum.
    // note we use 9500us here not 10000us
    // the read rate will end up at exactly 100hz because the Periodic Timer fires at 1khz
    if (tnow - _timer < 9500) {
        return;
    }

    _timer = tnow;

    if (_state == 0) {
        _s_DD2 += read_adc();// On state 0 we read temp
        _DD2_count++;
        if (_DD2_count == 32) {
            // we have summed 32 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_DD2 >>= 1;
            _DD2_count = 16;
        }
        _state++;
        write(CMD_CONVERT_DD1_OSR4096);      // Command to read pressure
    } else {
        _s_DD1 += read_adc();
        _DD1_count++;
        if (_DD1_count == 128) {
            // we have summed 128 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_DD1 >>= 1;
            _DD1_count = 64;
        }
        _state++;
        // Now a new reading exists
        _updated = true;
        if (_state == 5) {
            write(CMD_CONVERT_DD2_OSR4096); // Command to read temperature
            _state = 0;
        } else {
            write(CMD_CONVERT_DD1_OSR4096); // Command to read pressure
        }
    }
}

uint8_t read()
{
    bool updated = _updated;
    if (updated) {
        uint32_t sDD1, sDD2;
        uint8_t DD1count, DD2count;

        // Suspend timer procs because these variables are written to
        // in "_update".

        //hal.scheduler->suspend_timer_procs();
        sDD1 = _s_DD1; _s_DD1 = 0;
        sDD2 = _s_DD2; _s_DD2 = 0;
        DD1count = _DD1_count; _DD1_count = 0;
        DD2count = _DD2_count; _DD2_count = 0;
        _updated = false;
        //hal.scheduler->resume_timer_procs();

        if (DD1count != 0) {
            DD1 = ((float)sDD1) / DD1count;
        }
        if (DD2count != 0) {
            DD2 = ((float)sDD2) / DD2count;
        }
        _pressure_samples = DD1count;
        _raw_press = DD1;
        _raw_temp = DD2;
    }
    _calculate();
    if (updated) {
        _last_update = hal.scheduler->millis();
    }
    return updated ? 1 : 0;
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void _calculate()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;
    float P;

    // Formulas from manufacturer datasheet
    // sub -20c temperature compensation is not included

    // we do the calculations using floating point
    // as this is much faster on an AVR2560, and also allows
    // us to take advantage of the averaging of DD1 and DD1 over
    // multiple samples, giving us more precision
    dT = DD2-(((uint32_t)C5)<<8);
    TEMP = (dT * C6)/8388608;
    OFF = C2 * 65536.0f + (C4 * dT) / 128;
    SENS = C1 * 32768.0f + (C3 * dT) / 256;

    if (TEMP < 0) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = TEMP*TEMP;
        float OFF2 = 2.5f*Aux;
        float SENS2 = 1.25f*Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    P = (DD1*SENS/2097152 - OFF)/32768;
    Temp = TEMP + 2000;
    Press = P;
}

float get_pressure()
{
    return Press;
}

float get_temperature()
{
    // callers want the temperature in 0.1C units
    return Temp/10;
}

int32_t get_raw_pressure() {
    return _raw_press;
}

int32_t get_raw_temp() {
    return _raw_temp;
}


float get_altitude(void)
{
    float scaling, temp;

    if (_last_altitude_t == _last_update) {
        // no new information
        return _altitude;
    }

    // this has no filtering of the pressure values, use a separate
    // filter if you want a smoothed value. The AHRS driver wants
    // unsmoothed values
    scaling                                 = (float)_ground_pressure / (float)get_pressure();
    temp                                    = ((float)_ground_temperature) + 273.15f;
    _altitude = logf(scaling) * temp * 29.271267f;

    _last_altitude_t = _last_update;

    return _altitude;
}

static void setup() {
    hal.console->printf_P(PSTR("Initializing MS5611\r\n"));
    spidev = hal.spi->device(AP_HAL::SPIDevice_MS5611);

    ms5611_init();
}

static void loop() {

    _update(hal.scheduler->micros());
    read();

    int32_t alt = get_altitude();                 // calls barometer.read()

    float pres = get_pressure();
    int16_t temp = get_temperature();
    int32_t raw_pres = get_raw_pressure();
    int32_t raw_temp = get_raw_temp();
    hal.console->printf_P(PSTR("alt: %ldcm, pres: %fmbar, temp: %d/100degC,"
                         " raw pres: %ld, raw temp: %ld\n"),
                        (long)alt, pres, (int)temp, (long)raw_pres, (long)raw_temp);
    if( hal.console->available() > 0) {
        return (0);
    }


    hal.scheduler->delay(100);
}

AP_HAL_MAIN();
