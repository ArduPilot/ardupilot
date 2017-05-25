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

#include "AP_Telemetry.h"
#include "AP_Telemetry_Backend.h"
#include "AP_Telemetry_MQTT.h"

extern const AP_HAL::HAL& hal;

AP_Telemetry::AP_Telemetry()
{}

// perform required initialisation
void AP_Telemetry::init(const AP_SerialManager &serial_manager, const AP_AHRS &ahrs)
{
    // only perform once
    if (_num_instances > 0) {
        return;
    }

    // add pointer to ahrs
    _ahrs = &ahrs;

    // check for supported protocols
    AP_HAL::UARTDriver *uart;
    if ((uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MQTT, 0)) == nullptr) {
        _drivers[_num_instances] = AP_Telemetry_MQTT::init_telemetry_mqtt(*this, uart);
        _num_instances++;
    }
}

void AP_Telemetry::send_text(const char *str)
{
    for (uint8_t i=0; i<AP_TELEMETRY_MAX_INSTANCES; i++) {
        if (_drivers[i] != nullptr) {
            _drivers[i]->send_log(str);
        }
    }

}

int AP_Telemetry::recv_mavlink_message(mavlink_message_t *msg)
{
    int ret;
    ret = 0;
    for (uint8_t i=0; i<AP_TELEMETRY_MAX_INSTANCES; i++) {
        if (_drivers[i] != nullptr) {
            ret = _drivers[i]->recv_mavlink_message(msg);
        }
    }
    return ret;
}

// provide an opportunity to read/send telemetry
void AP_Telemetry::update()
{
    for (uint8_t i=0; i<AP_TELEMETRY_MAX_INSTANCES; i++) {
        if (_drivers[i] != nullptr) {
            _drivers[i]->update();
        }
    }
}
