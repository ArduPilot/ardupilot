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

#include "AP_AirSensor.h"

#include "AP_AirSensor_config.h"
#include "AP_Math/vector3.h"

class AP_AirSensor_Backend {
public:
    AP_AirSensor_Backend() = delete;
	AP_AirSensor_Backend(AP_AirSensor &frontend, AP_AirSensor::State &state, AP_AirSensor_Params &params);
    virtual bool init() = 0;
    virtual ~AP_AirSensor_Backend() {}
    virtual void update() = 0;
    // get measured wind vector [m/s]
    virtual bool get_wind(Vector3f& wind_uvw) const { return false; }
    // get measured angle of attack [rad]
    virtual bool get_aoa(float& a) const;

#if AP_SCRIPTING_ENABLED
    // this is in body frame
    virtual bool handle_script_3d_msg(const Vector3f &wind_uvw) { return false; }
#endif

    // return the type of sensor
    AP_AirSensor::Type type() const {
        // return (AP_Proximity::Type)params.type.get(); // TODO use the right type
        return  AP_AirSensor::Type::SCRIPTING;   
    }

protected:
    // set status and update valid_count
    void set_status(AP_AirSensor::Status status);

private:
    AP_AirSensor::Type _backend_type;

    AP_AirSensor &_frontend;
    AP_AirSensor::State &_state; // reference to this instances state
    AP_AirSensor_Params &_params; // parameters for this backend
};
