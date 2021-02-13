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
   IBus telemetry should always run at 115200 baud rate.
   It uses a half duplex protocol, so you need only to wire the TX pin to the telemetry port of the receiver
   You can configure via your preferred GCS the serial port to send IBus telemetry data
   You  need to initialize this object and then add sensors and set sensor measurements (about every 500ms - 1sec)
*/

#include <AP_IBus_Telem/AP_IBus_Telem.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_GPS/AP_GPS.h>

#if HAL_IBUS_TELEM_ENABLED

#define AP_SERIALMANAGER_IBUS_TELEM_BAUD 115200

void AP_IBus_Telem::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IBUS_Telem, 0))) {
        _port->set_options(_port->OPTION_HDPLEX);
        _port->begin(AP_SERIALMANAGER_IBUS_TELEM_BAUD);
        _initialized = true;
    }
}

// Populate the last two bytes of packet with the checksum of the preceeding size-2 bytes
void AP_IBus_Telem::populate_checksum(uint8_t *packet, uint16_t size)
{
    uint16_t checksum = 0xFFFF;

    for (int i=0; i<size-2; i++) {
        checksum -= packet[i];
    }

    packet[size-2] = checksum & 0x0ff;
    packet[size-1] = checksum >> 8;
}

void AP_IBus_Telem::loop(void)
{
    // no need to run the loop if not initialized or no sensors added
    if (!_initialized) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - _last_send_measurements_time > UPDATE_MEASUREMENTS_TIMING) {
        setSensorMeasurements();
    }

    // only process data already in our UART receive buffer
    uint16_t count = MAX(_port->available(), 1024U);

    for (uint16_t i=0; i<count; i++) {
        // only consider a new data package if we have not heard anything for >3ms
        if (now - _last >= PROTOCOL_TIMEGAP) {
            _state = State::GET_LENGTH;
        }
        _last = now;

        const int16_t c = _port->read();
        if (c < 0) {
            return;
        }

        switch (_state) {
        case State::GET_LENGTH:
            if (c <= PROTOCOL_MAX_LENGTH && c > PROTOCOL_OVERHEAD) {
                _offs = 0;
                _len = c - PROTOCOL_OVERHEAD;
                _chksum = 0xFFFF - c;
                _state = State::GET_DATA;
            } else {
                _state = State::DISCARD;
            }
            break;

        case State::GET_DATA:
            _buffer[_offs++] = c;
            _chksum -= c;
            if (_offs >= _len) {
                _state = State::GET_CHKSUML;
            }
            break;

        case State::GET_CHKSUML:
            _lchksum = c;
            _state = State::GET_CHKSUMH;
            break;

        case State::GET_CHKSUMH:
            // Validate checksum
            if (_chksum == (c << 8) + _lchksum) {
                handleTelemetryMessage();
            }
            break;
        case State::DISCARD:
            _state = State::GET_LENGTH;
            break;
        }
    }
}

void AP_IBus_Telem::handleTelemetryMessage()
{
    // Checksum is all fine Execute command -
    const uint8_t adr = _buffer[0] & 0x0f;

    // all sensor data commands go here
    // we only process the len==1 commands (=message length is 4 bytes incl overhead) to prevent the case the
    // return messages from the UART TX port loop back to the RX port and are processed again. This is extra
    // precaution as it will also be prevented by the PROTOCOL_TIMEGAP required
    if (adr > sizeof(_sensors) || adr == 0 || _len != 1) {
        return;
    }

    _sensorinfo *s = &_sensors[adr - 1];

    switch (_buffer[0] & 0x0f0) {
    case PROTOCOL_COMMAND_DISCOVER: { // 0x80, discover sensor
        // echo discover command: 0x04, 0x81, 0x7A, 0xFF
        struct protocol_command_discover_response_t {
            uint8_t command_length;
            uint8_t address;
            uint16_t checksum;
        } packet {
            PROTOCOL_FOUR_LENGTH,
            (uint8_t)(PROTOCOL_COMMAND_DISCOVER | adr)
        };
        populate_checksum((uint8_t*)&packet, sizeof(packet));
        _port->write((uint8_t*)&packet, sizeof(packet));
        break;
    }
    case PROTOCOL_COMMAND_TYPE: { // 0x90, send sensor type
        // echo sensortype command: 0x06 0x91 0x00 0x02 0x66 0xFF
        struct protocol_command_type_response_t {
            uint8_t command_length;
            uint8_t address;
            uint8_t type;
            uint8_t length;
            uint16_t checksum;
        } packet {
            PROTOCOL_SIX_LENGTH,
            (uint8_t)(PROTOCOL_COMMAND_TYPE | adr),
            s->sensor_type,
            s->sensor_length
        };
        populate_checksum((uint8_t*)&packet, sizeof(packet));
        _port->write((uint8_t*)&packet, sizeof(packet));
        break;
    }
    case PROTOCOL_COMMAND_VALUE: { // 0xA0, send sensor data
        struct protocol_command_2byte_value_response_t {
            uint8_t command_length;
            uint8_t address;
            uint16_t value;
            uint16_t checksum;
        } packet {
            PROTOCOL_SIX_LENGTH,
            (uint8_t)(PROTOCOL_COMMAND_VALUE | adr),
            (uint16_t)s->sensor_value
        };
        populate_checksum((uint8_t*)&packet, sizeof(packet));
        _port->write((uint8_t*)&packet, sizeof(packet));
        break;
    }
    default:
        break;
    }
}


void AP_IBus_Telem::setSensorMeasurements()
{
    // no need to do things if IBusTelemetry is not activated
    if (!_initialized) {
        return;
    }
    // sensors can easily be added if needed
    uint16_t voltage = (uint16_t)(AP::battery().voltage() * 100);
    _sensors[0].sensor_value =  voltage; // IBUS_MEAS_TYPE_EXTV
    _sensors[1].sensor_value = AP::arming().is_armed(); // IBUS_MEAS_TYPE_ARMED
    _sensors[2].sensor_value = (uint8_t)AP::vehicle()->get_mode(); // IBUS_MEAS_TYPE_FLIGHT_MODE
    _sensors[3].sensor_value = (uint8_t)AP::gps().is_healthy(); //IBUS_MEAS_TYPE_GPS_STATUS

}

#endif
