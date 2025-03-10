/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  backend driver for airspeed from a I2C AUAV sensor
 */
#include "AP_Airspeed_AUAV.h"

#if AP_AIRSPEED_AUAV_ENABLED

#define AUAV_AIRSPEED_I2C_ADDR 0x26

// the sensor supports a 2nd channel (2nd I2C address) for absolute pressure
// we could use this as a baro driver but don't yet
#define AUAV_AIRSPEED_I2C_ADDR_ABSOLUTE 0x27

// max measurement time for 8x averaging differential is 16.2ms
#define MEASUREMENT_TIME_MAX_MS 17

#define MEASUREMENT_TIMEOUT_MS 200

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;


AP_Airspeed_AUAV::AP_Airspeed_AUAV(AP_Airspeed &_frontend, uint8_t _instance, const float _range_inH2O) :
    AP_Airspeed_Backend(_frontend, _instance),
    range_inH2O(_range_inH2O)
{
}

// probe for a sensor
bool AP_Airspeed_AUAV::probe(uint8_t bus, uint8_t address)
{
    _dev = hal.i2c_mgr->get_device_ptr(bus, address);
    if (!_dev) {
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);
    _measure();
    hal.scheduler->delay(20);
    _collect();

    return last_sample_time_ms != 0;

}

// probe and initialise the sensor
bool AP_Airspeed_AUAV::init()
{
    const auto bus = get_bus();
    if (!probe(bus, AUAV_AIRSPEED_I2C_ADDR)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AUAV AIRSPEED[%u]: no sensor found on bus %u", get_instance(), bus);
        return false;
    }

    _dev->set_device_type(uint8_t(DevType::AUAV));
    set_bus_id(_dev->get_bus_id());

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV AIRSPEED[%u]: Found bus %u addr 0x%02x", get_instance(), _dev->bus_num(), _dev->get_bus_address());

    WITH_SEMAPHORE(_dev->get_semaphore());

    // drop to 2 retries for runtime
    _read_coefficients();
    _dev->set_retries(2);
    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_AUAV::_timer, void));
    return true;
}

// start a measurement
void AP_Airspeed_AUAV::_measure()
{
    measurement_started_ms = 0;
    uint8_t cmd = 0xAE; // start average8, max 16.2ms measurement time
    if (_dev->transfer(&cmd, 1, nullptr, 0)) {
        measurement_started_ms = AP_HAL::millis();
    }
}

// read the values from the sensor
void AP_Airspeed_AUAV::_collect()
{
    measurement_started_ms = 0; // It should always get reset by _measure. This is a safety to handle failures of i2c bus
    uint8_t inbuf[7];
    if (!_dev->read((uint8_t *)&inbuf, sizeof(inbuf))) {
        return;
    }
    const uint8_t status = inbuf[0];
    const uint8_t healthy_status = 0x50; // power on, not busy, differential normal, not out of range
    if (status != healthy_status) {
        // not ready or error
        return;
    }
    const int32_t Tref_Counts = 7576807; // temperature counts at 25C
    const float TC50Scale = 100.0f * 100.0f * 167772.2f; // scale TC50 to 1.0% FS0
    float Pdiff, TC50;

    // Convert unsigned 24-bit pressure value to signed +/- 23-bit:
    const int32_t iPraw = (inbuf[1]<<16) + (inbuf[2]<<8) + inbuf[3] - 0x800000;

    // Convert signed 23-bit valu11e to float, normalized to +/- 1.0:
    const float Pnorm = float(iPraw) / float(0x7FFFFF);
    const float AP3 = DLIN_A * Pnorm * Pnorm * Pnorm; // A*Pout^3
    const float BP2 = DLIN_B * Pnorm * Pnorm; // B*Pout^2
    const float CP = DLIN_C * Pnorm; // C*POut
    const float Corr = AP3 + BP2 + CP + DLIN_D; // Linearity correction term
    const float Pcorr = Pnorm + Corr; // Corrected P, range +/-1.0.

    // Compute difference from reference temperature, in sensor counts:
    const int32_t iTemp = (inbuf[4]<<16) + (inbuf[5]<<8) + inbuf[6]; // 24-bit temperature

    // get temperature in degrees C
    temp_C = (iTemp * 155) / float(1U<<24) - 45;

    const int32_t Tdiff = iTemp - Tref_Counts; // see constant defined above.
    const float Pnfso = (Pcorr + 1.0f) * 0.5;

    //TC50: Select High/Low, based on current temp above/below 25C:
    if (Tdiff > 0) {
        TC50 = D_TC50H;
    } else {
        TC50 = D_TC50L;
    }

    // Find absolute difference between midrange and reading (abs(Pnfso-0.5)):
    if (Pnfso > 0.5) {
        Pdiff = Pnfso - 0.5;
    } else {
        Pdiff = 0.5f - Pnfso;
    }

    const float Tcorr = (1.0f - (D_Es * 2.5f * Pdiff)) * Tdiff * TC50 / TC50Scale;
    const float PCorrt = Pnfso - Tcorr; // corrected P: float, [0 to +1.0)
    const uint32_t PComp = (uint32_t) (PCorrt * (float)0xFFFFFF);
    pressure_digital = PComp;

    last_sample_time_ms = AP_HAL::millis();
}

uint32_t AP_Airspeed_AUAV::_read_register(uint8_t cmd)
{
    uint8_t raw_bytes1[3];
    if (!_dev->transfer(&cmd,1,(uint8_t *)&raw_bytes1, sizeof(raw_bytes1))) {
        return 0;
    }
    uint8_t raw_bytes2[3];
    uint8_t cmd2 = cmd + 1;
    if (!_dev->transfer(&cmd2,1,(uint8_t *)&raw_bytes2, sizeof(raw_bytes2))) {
        return 0;
    }
    uint32_t result = ((uint32_t)raw_bytes1[1] << 24) | ((uint32_t)raw_bytes1[2] << 16) | ((uint32_t)raw_bytes2[1] << 8) | (uint32_t)raw_bytes2[2];
    return result;
}

bool AP_Airspeed_AUAV::_read_coefficients()
{
    // Differential Coefficients
    int32_t i32A = 0, i32B =0, i32C =0, i32D=0, i32TC50HLE=0;
    int8_t i8TC50H = 0, i8TC50L = 0, i8Es = 0;
    i32A = _read_register(0x2B);
    DLIN_A = ((float)(i32A))/((float)(0x7FFFFFFF));

    i32B = _read_register(0x2D);
    DLIN_B = (float)(i32B)/(float)(0x7FFFFFFF);

    i32C = _read_register(0x2F);
    DLIN_C = (float)(i32C)/(float)(0x7FFFFFFF);

    i32D = _read_register(0x31);
    DLIN_D = (float)(i32D)/(float)(0x7FFFFFFF);

    i32TC50HLE = _read_register(0x33);
    i8TC50H = (i32TC50HLE >> 24) & 0xFF; // 55 H
    i8TC50L = (i32TC50HLE >> 16) & 0xFF; // 55 L
    i8Es = (i32TC50HLE ) & 0xFF; // 56 L
    D_Es = (float)(i8Es)/(float)(0x7F);
    D_TC50H = (float)(i8TC50H)/(float)(0x7F);
    D_TC50L = (float)(i8TC50L)/(float)(0x7F);

    return true; //Need to actually check
}

// 50Hz timer
void AP_Airspeed_AUAV::_timer()
{
    if (measurement_started_ms == 0) {
        _measure();
    }
    if ((AP_HAL::millis() - measurement_started_ms) > MEASUREMENT_TIME_MAX_MS) {
        _collect();
        // start a new measurement
        _measure();
    }
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_AUAV::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);
    _pressure = 248.8f*1.25f*((pressure_digital-8388608)/16777216.0f) * 2 * range_inH2O;
    return AP_HAL::millis() - last_sample_time_ms < MEASUREMENT_TIMEOUT_MS;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_AUAV::get_temperature(float &_temperature)
{
    WITH_SEMAPHORE(sem);
    _temperature = temp_C;
    return AP_HAL::millis() - last_sample_time_ms < MEASUREMENT_TIMEOUT_MS;
}

#endif  // AP_AIRSPEED_AUAV_ENABLED
