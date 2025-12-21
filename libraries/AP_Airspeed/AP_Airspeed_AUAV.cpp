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
#define MEASUREMENT_TIMEOUT_MS 200

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AUAV_Pressure_sensor::AUAV_Pressure_sensor(AP_HAL::Device *&_dev, Type _type) :
    dev(_dev),
    type(_type)
{
}

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
    if (!sensor.measure()) {
        return false;
    }
    hal.scheduler->delay(20);

    // Don't fill in values yet as the compensation coefficients are not configured
    float PComp, temperature;
    return sensor.collect(PComp, temperature) == AUAV_Pressure_sensor::Status::Normal;
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

    // drop to 2 retries for runtime
    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_AUAV::_timer, void));
    return true;
}

// start a measurement
bool AUAV_Pressure_sensor::measure()
{
    uint8_t cmd = 0xAE; // start average8
    // Differential: max 16.2ms measurement time 
    // Absolute: 37ms
    if (dev->transfer(&cmd, 1, nullptr, 0)) {
        return true;
    }
    return false;
}

// read the values from the sensor
// Return true if successful
// pressure corrected but not scaled to the correct units
// temperature in degrees centigrade
AUAV_Pressure_sensor::Status AUAV_Pressure_sensor::collect(float &PComp, float &temperature)
{
    uint8_t inbuf[7];
    if (!dev->read((uint8_t *)&inbuf, sizeof(inbuf))) {
        return Status::Fault;
    }
    const uint8_t status = inbuf[0];

    // power on, not busy, reading normal, not out of range
    const uint8_t healthy_status = type == Type::Differential ? 0x50 :  0x40;
    if (status != healthy_status) {
        if ((status & (1U << 5)) != 0) {
            return Status::Busy;
        }
        return Status::Fault;
    }
    const int32_t Tref_Counts = 7576807; // temperature counts at 25C
    const float TC50Scale = 100.0f * 100.0f * 167772.2f; // scale TC50 to 1.0% FS0
    float Pdiff, TC50;

    // Convert unsigned 24-bit pressure value to signed +/- 23-bit:
    const int32_t iPraw = (inbuf[1]<<16) + (inbuf[2]<<8) + inbuf[3] - 0x800000;

    // Convert signed 23-bit value to float, normalized to +/- 1.0:
    const float Pnorm = float(iPraw) / float(0x7FFFFF);
    const float AP3 = LIN_A * Pnorm * Pnorm * Pnorm; // A*Pout^3
    const float BP2 = LIN_B * Pnorm * Pnorm; // B*Pout^2
    const float CP = LIN_C * Pnorm; // C*POut
    const float Corr = AP3 + BP2 + CP + LIN_D; // Linearity correction term
    const float Pcorr = Pnorm + Corr; // Corrected P, range +/-1.0.

    // Compute difference from reference temperature, in sensor counts:
    const int32_t iTemp = (inbuf[4]<<16) + (inbuf[5]<<8) + inbuf[6]; // 24-bit temperature

    // get temperature in degrees C
    temperature = (iTemp * 155) / float(1U<<24) - 45;

    const int32_t Tdiff = iTemp - Tref_Counts; // see constant defined above.
    const float Pnfso = (Pcorr + 1.0f) * 0.5;

    //TC50: Select High/Low, based on current temp above/below 25C:
    if (Tdiff > 0) {
        TC50 = TC50H;
    } else {
        TC50 = TC50L;
    }

    // Find absolute difference between midrange and reading (abs(Pnfso-0.5)):
    if (Pnfso > 0.5) {
        Pdiff = Pnfso - 0.5;
    } else {
        Pdiff = 0.5f - Pnfso;
    }

    const float Tcorr = (1.0f - (Es * 2.5f * Pdiff)) * Tdiff * TC50 / TC50Scale;
    const float PCorrt = Pnfso - Tcorr; // corrected P: float, [0 to +1.0)
    PComp = PCorrt * (float)0xFFFFFF;

    return Status::Normal;
}

bool AUAV_Pressure_sensor::read_register(uint8_t cmd, uint16_t &result)
{
    switch (coefficient_step) {
        case CoefficientStep::request: {
            if (dev->transfer(&cmd, sizeof(cmd), nullptr, 0)) {
                coefficient_step = CoefficientStep::read;
            }
            return false;
        }

        case CoefficientStep::read: {
            // Next step is always another request, either for the next coefficient or a re-request if this read fails
            coefficient_step = CoefficientStep::request;

            const uint8_t healthy_status = type == Type::Differential ? 0x50 :  0x40;
            uint8_t raw_bytes[3];
            if (!dev->read((uint8_t *)&raw_bytes, sizeof(raw_bytes))) {
                return false;
            }
            if (raw_bytes[0] != healthy_status) {
                return false;
            }
            result = ((uint16_t)raw_bytes[1] << 8) | (uint16_t)raw_bytes[2];
            return true;
        }
    }

    return false;

}

void AUAV_Pressure_sensor::read_coefficients()
{
    const uint8_t cmd_start = type == Type::Differential ? 0x2B : 0x2F;

    // Differential can happily run through these in one call, Absolute gets stuck, no idea why.
    // Space them out anyway to be inline with the spec

    switch (stage) {
    case CoefficientStage::A_high: {
        uint16_t high;
        if (!read_register(cmd_start + 0, high)) {
            return;
        }
        LIN_A = (float)(high << 16)/(float)(0x7FFFFFFF);
        stage = CoefficientStage::A_low;
        break;
    }

    case CoefficientStage::A_low: {
        uint16_t low;
        if (!read_register(cmd_start + 1, low)) {
            return;
        }
        LIN_A += (float)(low)/(float)(0x7FFFFFFF);
        stage = CoefficientStage::B_high;
        break;
    }

    case CoefficientStage::B_high: {
        uint16_t high;
        if (!read_register(cmd_start + 2, high)) {
            return;
        }
        LIN_B = (float)(high << 16)/(float)(0x7FFFFFFF);
        stage = CoefficientStage::B_low;
        break;
    }

    case CoefficientStage::B_low: {
        uint16_t low;
        if (!read_register(cmd_start + 3, low)) {
            return;
        }
        LIN_B += (float)(low)/(float)(0x7FFFFFFF);
        stage = CoefficientStage::C_high;
        break;
    }

    case CoefficientStage::C_high: {
        uint16_t high;
        if (!read_register(cmd_start + 4, high)) {
            return;
        }
        LIN_C = (float)(high << 16)/(float)(0x7FFFFFFF);
        stage = CoefficientStage::C_low;
        break;
    }

    case CoefficientStage::C_low: {
        uint16_t low;
        if (!read_register(cmd_start + 5, low)) {
            return;
        }
        LIN_C += (float)(low)/(float)(0x7FFFFFFF);
        stage = CoefficientStage::D_high;
        break;
    }

    case CoefficientStage::D_high: {
        uint16_t high;
        if (!read_register(cmd_start + 6, high)) {
            return;
        }
        LIN_D = (float)(high << 16)/(float)(0x7FFFFFFF);
        stage = CoefficientStage::D_low;
        break;
    }

    case CoefficientStage::D_low: {
        uint16_t low;
        if (!read_register(cmd_start + 7, low)) {
            return;
        }
        LIN_D += (float)(low)/(float)(0x7FFFFFFF);
        stage = CoefficientStage::E_high;
        break;
    }

    case CoefficientStage::E_high: {
        uint16_t TC50HL;
        if (!read_register(cmd_start + 8, TC50HL)) {
            return;
        }
        const int8_t i8TC50H = (TC50HL >> 8) & 0xFF;
        const int8_t i8TC50L = TC50HL & 0xFF;
        TC50H = (float)(i8TC50H)/(float)(0x7F);
        TC50L = (float)(i8TC50L)/(float)(0x7F);
        stage = CoefficientStage::E_low;
        break;
    }

    case CoefficientStage::E_low: {
        uint16_t es;
        if (!read_register(cmd_start + 9, es)) {
            return;
        }
        Es = (float)(es & 0xFF)/(float)(0x7F);
        stage = CoefficientStage::Done;

        // Drop to 2 retry's for runtime
        dev->set_retries(2);
        break;
    }

    case CoefficientStage::Done:
        break;
    }
}

void AP_Airspeed_AUAV::_timer()
{
    if (sensor.stage != AUAV_Pressure_sensor::CoefficientStage::Done) {
        sensor.read_coefficients();
        return;
    }

    if (measurement_requested) {
        // Read in result of last measurement
        float Pcomp;
        switch (sensor.collect(Pcomp, temp_C)) {
        case AUAV_Pressure_sensor::Status::Normal:
            // Calculate pressure and update last read time
            last_sample_time_ms = AP_HAL::millis();

            // Convert to correct units and apply range
            pressure = 248.8f*1.25f*((Pcomp-8388608)/16777216.0f) * 2 * range_inH2O;
            break;

        case AUAV_Pressure_sensor::Status::Busy:
            // Don't request a new measurement
            return;

        case AUAV_Pressure_sensor::Status::Fault:
            break;
        }
    }

    // Request a new measurement
    measurement_requested = sensor.measure();
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_AUAV::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);
    _pressure = pressure;
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
