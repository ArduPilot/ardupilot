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

#include "AP_Proximity.h"
#include "AP_Proximity_LightWareSF40C.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Proximity::var_info[] = {
    // 0 is reserved for possible addition of an ENABLED parameter

    // @Param: _TYPE
    // @DisplayName: Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,1:LightWareSF40C
    // @User: Standard
    AP_GROUPINFO("_TYPE",   1, AP_Proximity, _type[0], 0),

    // @Param: _ORIENT
    // @DisplayName: Proximity sensor orientation
    // @Description: Proximity sensor orientation
    // @Values: 0:Default,1:Upside Down
    // @User: Standard
    AP_GROUPINFO("_ORIENT", 2, AP_Proximity, _orientation[0], 0),

    // @Param: _YAW_CORR
    // @DisplayName: Proximity sensor yaw correction
    // @Description: Proximity sensor yaw correction
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("_YAW_CORR", 3, AP_Proximity, _yaw_correction[0], PROXIMITY_YAW_CORRECTION_DEFAULT),

#if PROXIMITY_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,1:LightWareSF40C
    // @User: Advanced
    AP_GROUPINFO("2_TYPE", 4, AP_Proximity, _type[1], 0),

    // @Param: _ORIENT
    // @DisplayName: Second Proximity sensor orientation
    // @Description: Second Proximity sensor orientation
    // @Values: 0:Default,1:Upside Down
    // @User: Standard
    AP_GROUPINFO("2_ORIENT", 5, AP_Proximity, _orientation[1], 0),

    // @Param: _YAW_CORR
    // @DisplayName: Second Proximity sensor yaw correction
    // @Description: Second Proximity sensor yaw correction
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("2_YAW_CORR", 6, AP_Proximity, _yaw_correction[1], PROXIMITY_YAW_CORRECTION_DEFAULT),
#endif

    AP_GROUPEND
};

AP_Proximity::AP_Proximity(AP_SerialManager &_serial_manager) :
    primary_instance(0),
    num_instances(0),
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the Proximity class. We do detection of attached sensors here
// we don't allow for hot-plugging of sensors (i.e. reboot required)
void AP_Proximity::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<PROXIMITY_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != NULL) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }

        // initialise status
        state[i].status = Proximity_NotConnected;
    }
}

// update Proximity state for all instances. This should be called at a high rate by the main loop
void AP_Proximity::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != NULL) {
            if (_type[i] == Proximity_Type_None) {
                // allow user to disable a proximity sensor at runtime
                state[i].status = Proximity_NotConnected;
                continue;
            }
            drivers[i]->update();
        }
    }

    // work out primary instance - first sensor returning good data
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != NULL && (state[i].status == Proximity_Good)) {
            primary_instance = i;
        }
    }
}

// return sensor orientation
uint8_t AP_Proximity::get_orientation(uint8_t instance) const
{
    if (instance >= PROXIMITY_MAX_INSTANCES) {
        return 0;
    }

    return _orientation[instance].get();
}

// return sensor yaw correction
int16_t AP_Proximity::get_yaw_correction(uint8_t instance) const
{
    if (instance >= PROXIMITY_MAX_INSTANCES) {
        return 0;
    }

    return _yaw_correction[instance].get();
}

// return sensor health
AP_Proximity::Proximity_Status AP_Proximity::get_status(uint8_t instance) const
{
    // sanity check instance number
    if (instance >= num_instances) {
        return Proximity_NotConnected;
    }

    return state[instance].status;
}

AP_Proximity::Proximity_Status AP_Proximity::get_status() const
{
    return get_status(primary_instance);
}

//  detect if an instance of a proximity sensor is connected.
void AP_Proximity::detect_instance(uint8_t instance)
{
    uint8_t type = _type[instance];
    if (type == Proximity_Type_SF40C) {
        if (AP_Proximity_LightWareSF40C::detect(serial_manager)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_LightWareSF40C(*this, state[instance], serial_manager);
            return;
        }
    }
}

// get distance in meters in a particular direction in degrees (0 is forward, clockwise)
// returns true on successful read and places distance in distance
bool AP_Proximity::get_horizontal_distance(uint8_t instance, float angle_deg, float &distance) const
{
    if ((drivers[instance] == NULL) || (_type[instance] == Proximity_Type_None)) {
        return false;
    }
    // get distance from backend
    return drivers[instance]->get_horizontal_distance(angle_deg, distance);
}

// get distance in meters in a particular direction in degrees (0 is forward, clockwise)
// returns true on successful read and places distance in distance
bool AP_Proximity::get_horizontal_distance(float angle_deg, float &distance) const
{
    return get_horizontal_distance(primary_instance, angle_deg, distance);
}
