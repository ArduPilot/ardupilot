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

#include "AP_Generator.h"

#if HAL_GENERATOR_ENABLED

#include "AP_Generator_IE_650_800.h"
#include "AP_Generator_IE_2400.h"
#include "AP_Generator_RichenPower.h"

#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_Generator::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Generator type
    // @Description: Generator type
    // @Values: 0:Disabled, 1:IE 650w 800w Fuel Cell, 2:IE 2.4kW Fuel Cell, 3: Richenpower
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_Generator, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: OPTIONS
    // @DisplayName: Generator Options
    // @Description: Bitmask of options for generators
    // @Bitmask: 0:Supress Maintenance-Required Warnings
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 2, AP_Generator, _options, 0),

    AP_GROUPEND
};

// Constructor
AP_Generator::AP_Generator()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many generators");
#endif
        return;
    }
    _singleton = this;
}

void AP_Generator::init()
{
    // Select backend
    switch (type()) {
        case Type::GEN_DISABLED:
            // Not using a generator
            return;

        case Type::IE_650_800:
            _driver_ptr = new AP_Generator_IE_650_800(*this);
            break;

        case Type::IE_2400:
            _driver_ptr = new AP_Generator_IE_2400(*this);
            break;

        case Type::RICHENPOWER:
            _driver_ptr = new AP_Generator_RichenPower(*this);
            break;
    }

    if (_driver_ptr != nullptr) {
        _driver_ptr->init();
    }
}

void AP_Generator::update()
{
    // Return immediatly if not enabled. Don't support run-time disabling of generator
    if (_driver_ptr == nullptr) {
        return;
    }

    // Calling backend update will cause backend to update the front end variables
    _driver_ptr->update();
}

// Helper to get param and cast to Type
enum AP_Generator::Type AP_Generator::type() const
{
    return (Type)_type.get();
}

// Pass through to backend
void AP_Generator::send_generator_status(const GCS_MAVLINK &channel)
{
    if (_driver_ptr == nullptr) {
        return;
    }
    _driver_ptr->send_generator_status(channel);
}

// Tell backend to perform arming checks
bool AP_Generator::pre_arm_check(char* failmsg, uint8_t failmsg_len) const
{
    if (type() == Type::GEN_DISABLED) {
        // Don't prevent arming if generator is not enabled and has never been init
        if (_driver_ptr == nullptr) {
            return true;
        }
        // Don't allow arming if we have disabled the generator since boot
        strncpy(failmsg, "Generator disabled, reboot reqired", failmsg_len);
        return false;
    }
    if (_driver_ptr == nullptr) {
        strncpy(failmsg, "No backend driver", failmsg_len);
        return false;
    }
    return _driver_ptr->pre_arm_check(failmsg, failmsg_len);
}

// Tell backend check failsafes
AP_BattMonitor::Failsafe AP_Generator::update_failsafes()
{
    // Don't invoke a failsafe if driver not assigned
    if (_driver_ptr == nullptr) {
        return AP_BattMonitor::Failsafe::None;
    }
    return _driver_ptr->update_failsafes();
}

// Pass through to backend
bool AP_Generator::stop()
{
    // Still allow 
    if (_driver_ptr == nullptr) {
        return false;
    }
    return _driver_ptr->stop();
}

// Pass through to backend
bool AP_Generator::idle()
{
    if (_driver_ptr == nullptr) {
        return false;
    }
    return _driver_ptr->idle();
}

// Pass through to backend
bool AP_Generator::run()
{
    // Don't allow operators to request generator be set to run if it has been disabled
    if (_driver_ptr == nullptr) {
        return false;
    }
    return _driver_ptr->run();
}

// Get the AP_Generator singleton
AP_Generator *AP_Generator::get_singleton()
{
    return _singleton;
}

AP_Generator *AP_Generator::_singleton = nullptr;

namespace AP {
    AP_Generator *generator()
    {
        return AP_Generator::get_singleton();
    }
};
#endif
