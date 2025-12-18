/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andy Piper <github@andypiper.com>
 */

#include "AP_CRSF_config.h"

#if AP_CRSF_OUT_ENABLED

#include "AP_CRSF_OutManager.h"
#include "AP_CRSF_Out.h"
#include "AP_CRSF_Protocol.h"
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#define DEFAULT_CRSF_OUTPUT_RATE      250U // equivalent to tracer

const AP_Param::GroupInfo AP_CRSF_OutManager::var_info[] = {
    // @Param: RATE
    // @DisplayName: CRSF output rate
    // @Description: This sets the CRSF output frame rate in Hz for RC Out.
    // @Range: 25 1000
    // @User: Advanced
    // @Units: Hz
    AP_GROUPINFO("RATE",  1, AP_CRSF_OutManager, _rate_hz, DEFAULT_CRSF_OUTPUT_RATE),

    // @Param: RPT_HZ
    // @DisplayName: CRSF output reporting rate
    // @Description: This sets the CRSF output reporting rate in Hz. 0 disables reporting.
    // @Range: 0 5
    // @User: Advanced
    // @Units: Hz
    AP_GROUPINFO("RPT_HZ",  2, AP_CRSF_OutManager, _reporting_rate_hz, 0),
    AP_GROUPEND
};

AP_CRSF_OutManager::AP_CRSF_OutManager(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

AP_CRSF_Out* AP_CRSF_OutManager::create_instance(AP_HAL::UARTDriver& uart)
{
#if AP_CRSF_OUT_ENABLED
    AP_CRSF_Out* crsf_out = NEW_NOTHROW AP_CRSF_Out(uart, _num_instances, *this);
    if (crsf_out == nullptr) {
        return nullptr;
    }
    return crsf_out;
#else
    return nullptr;
#endif
}

// callback from the RC thread
void AP_CRSF_OutManager::update()
{
    // give all the other instances a chance to run
    for (uint8_t i = 0; i < _num_instances; i++) {
        if (_instances[i] != nullptr) {
            _instances[i]->update();
        }
    }
}

AP_CRSF_OutManager::~AP_CRSF_OutManager()
{
    for (uint8_t i = 0; i < _num_instances; i++) {
        delete _instances[i];
        _instances[i] = nullptr;
    }
}

// init to find all configured CRSF ports, called at 1Hz from AP_Vehicle
void AP_CRSF_OutManager::init()
{
    AP_SerialManager &serial_manager = AP::serialmanager();

    for (uint8_t i = _num_instances; i < SERIALMANAGER_MAX_PORTS; i++) {
        if (_instances[i] != nullptr) {
            continue;
        }

        const AP_SerialManager::UARTState* port_state = serial_manager.get_state_by_id(i);

        if (port_state == nullptr) {
            continue;
        }

        AP_SerialManager::SerialProtocol protocol = port_state->get_protocol();
        AP_HAL::UARTDriver* uart = serial_manager.get_serial_by_id(i);

        if (uart == nullptr) {
            continue;
        }

        if (protocol == AP_SerialManager::SerialProtocol_CRSF) {
            _instances[i] = NEW_NOTHROW AP_RCProtocol_CRSF(AP::RC(), AP_RCProtocol_CRSF::PortMode::DIRECT_VTX, uart);
            _num_instances++;
#if AP_CRSF_OUT_ENABLED
        } else if (protocol == AP_SerialManager::SerialProtocol_CRSF_Output) {
            _instances[i] = create_instance(*uart);
            _num_instances++;
        }
#endif
    }
}


#endif // AP_CRSF_OUT_ENABLED