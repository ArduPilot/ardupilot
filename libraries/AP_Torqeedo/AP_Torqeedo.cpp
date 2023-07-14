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

#include "AP_Torqeedo.h"

#if HAL_TORQEEDO_ENABLED
#include "AP_Torqeedo_Backend.h"
#include "AP_Torqeedo_TQBus.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Torqeedo::var_info[] = {
    // @Group: 1_
    // @Path: AP_Torqeedo_Params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 25, AP_Torqeedo, AP_Torqeedo_Params),

    // @Group: 1_
    // @Path: AP_Torqeedo_TQBus.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, AP_Torqeedo, backend_var_info[0]),

#if TORQEEDO_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_Torqeedo_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, AP_Torqeedo, AP_Torqeedo_Params),

    // @Group: 2_
    // @Path: AP_Torqeedo_TQBus.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, AP_Torqeedo, backend_var_info[1]),
#endif

#if TORQEEDO_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_Torqeedo_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, AP_Torqeedo, AP_Torqeedo_Params),

    // @Group: 3_
    // @Path: AP_Torqeedo_TQBus.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  59, AP_Torqeedo, backend_var_info[2]),
#endif

   AP_GROUPEND
};

const AP_Param::GroupInfo *AP_Torqeedo::backend_var_info[TORQEEDO_MAX_INSTANCES];


AP_Torqeedo::AP_Torqeedo()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Torqeedo must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}


// detect and initialise any available propellers
void AP_Torqeedo::init()
{
	if (num_instances != 0) {
        // don't re-init if we've found some sensors already
        return;
    }

	// instantiate backends
	uint8_t serial_instance = 0;
	(void)serial_instance;  // in case no serial backends are compiled in
	for (uint8_t instance = 0; instance < TORQEEDO_MAX_INSTANCES; ++instance) {
		switch (get_type(instance)) {
		case Type::None:
			break;
#if AP_TORQEEDO_TQBUS_ENABLED
		case Type::TQBus:
			if (AP_Torqeedo_TQBus::detect(serial_instance)) {
				state[instance].instance = instance;
				drivers[instance] = new AP_Torqeedo_TQBus(*this, state[instance], params[instance], serial_instance);
				serial_instance++;
			}
		break;
#endif
		}
		
		if (drivers[instance] != nullptr) {
			// we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
			drivers[instance]->init();
            num_instances = instance+1;
		}

		// initialise status
        state[instance].status = Status::NotConnected;
	}
}

// update state of all propellers. Should be called at high rate from main loop
void AP_Torqeedo::update()
{
	for (uint8_t i=0; i<num_instances; i++) {
        if (!valid_instance(i)) {
            continue;
        }
        drivers[i]->update();
    }

// todo: log 
}


// return sensor type of a given instance
AP_Torqeedo::Type AP_Torqeedo::get_type(uint8_t instance) const
{
    if (instance < TORQEEDO_MAX_INSTANCES) {
        return (Type)((uint8_t)params[instance].type);
    }
    return Type::None;
}

// return sensor health
AP_Torqeedo::Status AP_Torqeedo::get_instance_status(uint8_t instance) const
{
	// sanity check instance number
    if (!valid_instance(instance)) {
        return Status::NotConnected;
    }

    return state[instance].status;
}

AP_Torqeedo::Status AP_Torqeedo::get_status() const
{
	for (uint8_t i=0; i<num_instances; i++) {
        const Status sensors_status = get_instance_status(i);
        if (sensors_status != Status::Good) {
            // return first bad status
            return sensors_status;
        }
    }
    // All valid sensors seem to be working
    return Status::Good;
}

// prearm checks
bool AP_Torqeedo::pre_arm_checks(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        switch (get_instance_status(i)) {
        case Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "TRQ%d: No Data", i + 1);
            return false;
        case Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "TRQ%d: Not Connected", i + 1);
            return false;
        case Status::Good:
            break;
        }
    }
    return true;
}

// get latest battery status info.  returns true on success and populates arguments
bool AP_Torqeedo::get_batt_info(uint8_t instance, float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const
{
	if (valid_instance(instance)) {
		return drivers[instance]->get_batt_info(voltage, current_amps, temp_C, pct_remaining);
	}
	return false;
}

bool AP_Torqeedo::get_batt_capacity_Ah(uint8_t instance, uint16_t &amp_hours) const
{
	if (valid_instance(instance)) {
		return drivers[instance]->get_batt_capacity_Ah(amp_hours);
	}
	return false;
}

void AP_Torqeedo::clear_motor_error()
{
	for (uint8_t i=0; i<num_instances; i++) {
        if (valid_instance(i)) {
			drivers[i]->clear_motor_error();
		}
	}
}


// return true if the given instance exists
bool AP_Torqeedo::valid_instance(uint8_t i) const
{
    if (i >= TORQEEDO_MAX_INSTANCES) {
        return false;
    }

    if (drivers[i] == nullptr) {
        return false;
    }
    return (Type)params[i].type.get() != Type::None;
}


AP_Torqeedo *AP_Torqeedo::_singleton;

namespace AP {

AP_Torqeedo *torqeedo()
{
    return AP_Torqeedo::get_singleton();
}

}

#endif