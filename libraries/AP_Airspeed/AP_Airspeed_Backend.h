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
#pragma once

/*
  backend driver class for airspeed
 */

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include "AP_Airspeed.h"
#include <AP_MSP/msp_sensors.h>

class AP_Airspeed_Backend {
public:
    AP_Airspeed_Backend(AP_Airspeed &frontend, uint8_t instance);
    virtual ~AP_Airspeed_Backend() {}

    // probe and initialise the sensor
    virtual bool init(void) = 0;

    // return the current differential_pressure in Pascal
    virtual bool get_differential_pressure(float &pressure) {return false;}

    // return the current temperature in degrees C, if available
    virtual bool get_temperature(float &temperature) = 0;

    // true if sensor reads airspeed directly, not via pressure
    virtual bool has_airspeed() {return false;}

    // return airspeed in m/s if available
    virtual bool get_airspeed(float& airspeed) {return false;}

    virtual void handle_msp(const MSP::msp_airspeed_data_message_t &pkt) {}
#if AP_AIRSPEED_EXTERNAL_ENABLED
    virtual void handle_external(const AP_ExternalAHRS::airspeed_data_message_t &pkt) {}
#endif

#if AP_AIRSPEED_HYGROMETER_ENABLE
    // optional hygrometer support
    virtual bool get_hygrometer(uint32_t &last_sample_ms, float &temperature, float &humidity) { return false; }
#endif

protected:
    int8_t get_pin(void) const;
    float get_psi_range(void) const;
    uint8_t get_bus(void) const;
    bool bus_is_configured(void) const;
    uint8_t get_instance(void) const {
        return instance;
    }

    // see if voltage correction should be disabled
    bool disable_voltage_correction(void) const {
        return (frontend._options.get() & AP_Airspeed::OptionsMask::DISABLE_VOLTAGE_CORRECTION) != 0;
    }

    AP_Airspeed::pitot_tube_order get_tube_order(void) const {
#ifndef HAL_BUILD_AP_PERIPH
        return AP_Airspeed::pitot_tube_order(frontend.param[instance].tube_order.get());
#else
        return AP_Airspeed::pitot_tube_order::PITOT_TUBE_ORDER_AUTO;
#endif
    }

    // semaphore for access to shared frontend data
    HAL_Semaphore sem;

    float get_airspeed_ratio(void) const {
        return frontend.get_airspeed_ratio(instance);
    }

    // some sensors use zero offsets
    void set_use_zero_offset(void) {
        frontend.state[instance].cal.state = AP_Airspeed::CalibrationState::NOT_REQUIRED_ZERO_OFFSET;
#ifndef HAL_BUILD_AP_PERIPH
        frontend.param[instance].offset.set(0.0);
#endif
    }

    // set use
    void set_use(int8_t use) {
#ifndef HAL_BUILD_AP_PERIPH
        frontend.param[instance].use.set(use);
#endif
    }

    // set bus ID of this instance, for ARSPD_DEVID parameters
    void set_bus_id(uint32_t id);

    enum class DevType {
        SITL     = 0x01,
        MS4525   = 0x02,
        MS5525   = 0x03,
        DLVR     = 0x04,
        MSP      = 0x05,
        SDP3X    = 0x06,
        UAVCAN   = 0x07,
        ANALOG   = 0x08,
        NMEA     = 0x09,
        ASP5033  = 0x0A,
        AUAV     = 0x0B,
    };
    
private:
    AP_Airspeed &frontend;
    uint8_t instance;
};

#endif  // AP_AIRSPEED_ENABLED
