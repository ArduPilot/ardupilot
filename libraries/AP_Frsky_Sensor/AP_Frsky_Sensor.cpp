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
   FrSky Sensor library

   Currently supported devices:
   - FLVSS
*/

#include "AP_Frsky_Sensor.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <math.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

AP_Frsky_Sensor *AP_Frsky_Sensor::singleton;

AP_Frsky_Sensor::AP_Frsky_Sensor()
{
    singleton = this;
}

AP_Frsky_Sensor::~AP_Frsky_Sensor(void)
{
    singleton = nullptr;
}

/*
 * init - perform required initialisation
 */
bool AP_Frsky_Sensor::init()
{
    if(singleton == NULL) {
        init();
    }

    const AP_SerialManager &serial_manager = AP::serialmanager();

    gcs().send_text(MAV_SEVERITY_INFO, "Initializing FrSky Sensors");

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort_Sensor_Input, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_SPort_Sensor_Input; // FrSky SPort protocol (X-receivers)
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }

    for(uint8_t index=0; index < FRSKY_MAX_FLVSS_INSTANCES; index++) {
        for(uint8_t j=0; j < FRSKY_FLVSS_MAX_CELLS; j++) {
            _FLVSS_inst[index].cell[j] = 0;
        }
    }
    return true;
}

/*
  thread to loop handling bytes
 */
void AP_Frsky_Sensor::loop(void)
{

    // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
    // assuming this is correct based upon the FRSKY Telem library.....  Not sure if I really need the begin call every loop call, surely it can be done in the init call?
    if (_protocol == AP_SerialManager::SerialProtocol_FrSky_SPort_Sensor_Input) {
        _port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        read_SPort_Sensors(AP_HAL::millis());
    } else {
        return;
    }
 }

float AP_Frsky_Sensor::get_SPort_voltage(uint16_t id)
{
    uint8_t index = getFLVSS_inst(id);

    return _FLVSS_inst[index].voltage;
}

uint32_t AP_Frsky_Sensor::get_SPort_read_time(uint16_t id)
{
    uint8_t index = getFLVSS_inst(id);

    return _FLVSS_inst[index].last_reading_ms;
}

uint8_t AP_Frsky_Sensor::get_SPort_num_cells(uint16_t id)
{
    uint8_t index = getFLVSS_inst(id);

    return _FLVSS_inst[index].num_cells;
}

uint16_t AP_Frsky_Sensor::get_SPort_cells(uint16_t id, uint8_t cell_num)
{
    uint8_t index = getFLVSS_inst(id);

    return _FLVSS_inst[index].cell[cell_num];

}

// Read Sensors
void AP_Frsky_Sensor::read_SPort_Sensors(uint32_t now)
{
    //loop through the various sensors we want to poll.
    uint8_t frsky_ids[28] = {0x00,0xA1,0x22,0x83,0xE4,0x45,0xC6,0x67,0x48,0xE9,0x6A,0xCB,0xAC,0x0D,0x8E,0x2F,0xD0,0x71,0xF2,0x53,0x34,0x95,0x16,0xB7,0x98,0x39,0xBA,0x1B};

    bool frame_header = false;
    bool match = false;
    uint16_t numc;

    uint8_t next_deviceID;

    if(_num_devices_found) {
        if(_loop_counter%2) {
            next_deviceID = _discovered_ids_list[_device_discovered_loop_count];
            _device_discovered_loop_count++;
        } else {
            next_deviceID = frsky_ids[_deviceID_loop_count];
            _deviceID_loop_count++;
        }
    } else {
        next_deviceID = frsky_ids[_deviceID_loop_count];
        _deviceID_loop_count++;
    }
    _loop_counter++;

    send_byte(0x7E);
    send_byte(next_deviceID);

    if(_deviceID_loop_count > 28)
        _deviceID_loop_count=0;

    // reset the device loop count to 0.
    if(_device_discovered_loop_count > (_num_devices_found-1))
        _device_discovered_loop_count = 0;

    numc = _port->available();

    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }

    while(numc) {
        // set receiving to true as we do not want to have
        _receiving = true;
        uint8_t data = read_byte();
        // Frame header, need to decode the following data otherwise discard the byte and continue till frame header is found
        if(data == 0x10) {
            frame_header = true;
            // if not in the list then increment and add to the list, if in the list continue on.
            match=false;
            for(uint8_t i = 0;i < _num_devices_found;i++) {
                if((_discovered_ids_list[i] == _previous_byte) & (_previous_byte != 0)) {
                    match = true;
                    break;
                }
            }

            if(!match) {
                _discovered_ids_list[_num_devices_found] = _previous_byte;
                _num_devices_found++;
                gcs().send_text(MAV_SEVERITY_INFO, "Frsky: New Discovered ID: 0x%x | Num: %d", _previous_byte, _num_devices_found);
            } else {
                match = false;
            }

            break;
        }
        _previous_byte = data;
        numc--;
    }

    numc = _port->available();

    // Wait till we have the id, payload and the crc.
    if(numc >= 7 && frame_header) {
        uint16_t id = 0;
        uint32_t payload = 0;
        uint8_t crc = 0;

        id = read_uint16();
        payload = read_uint32();
        crc = read_byte();
        if(crc != 0) {
            // Should prob check the crc value and exit safely if invalid - low risk not too yet as its not transmitted over the 'air'
            // it is a wired connection to the flight controller.
        }


        if(id >= 0x0300 && id <= 0x030F) {
            // Break down the payload into the different elements
            uint8_t batt_number = (payload & 0xF);
            uint8_t num_cells = (payload & 0xF0) >> 4;
            uint32_t volt1 = (payload & 0x000FFF00) >> 8;
            uint32_t volt2 = (payload & 0xFFF00000) >> 20;

            // Get the index of the current item, will return 255 if it cant find a valid value.
            uint8_t index = getFLVSS_inst(id);
            if(index != 255) {
                _FLVSS_inst[index].num_cells = num_cells;
                if(batt_number == 0) {
                    _FLVSS_inst[index].cell[0] = volt1 / 5;
                    _FLVSS_inst[index].cell[1] = volt2 / 5;
                } else if (batt_number == 2) {
                    _FLVSS_inst[index].cell[2] = volt1 / 5;
                    _FLVSS_inst[index].cell[3] = volt2 / 5;
                } else if (batt_number == 4) {
                    _FLVSS_inst[index].cell[4] = volt1 / 5;
                    _FLVSS_inst[index].cell[5] = volt2 / 5;
                }

                _FLVSS_inst[index].voltage = (_FLVSS_inst[index].cell[0] + \
                    _FLVSS_inst[index].cell[1] + \
                    _FLVSS_inst[index].cell[2] + \
                    _FLVSS_inst[index].cell[3] + \
                    _FLVSS_inst[index].cell[4] + \
                    _FLVSS_inst[index].cell[5]) / 100.0f;

                _FLVSS_inst[index].last_reading_ms = now;
            } else {
                // This shouldnt occur and if it does then it needs to be flagged on the GCS to warn operator.
                gcs().send_text(MAV_SEVERITY_WARNING, "Frsky: No sensor match found: 0x%x", id);
            }
        }
    }
}

// Find the matching id - Source ID will likely come from the battery serial id
uint8_t AP_Frsky_Sensor::getFLVSS_inst(uint16_t id) {
    for(uint8_t i=0;i < FRSKY_MAX_FLVSS_INSTANCES;i++) {
        if(_FLVSS_inst[i].id == id and _FLVSS_inst[i].id != 0) {
            return i;
        } else if (_FLVSS_inst[i].id == 0) {
            _FLVSS_inst[i].id = id;
            return i;
        }
    }
    return 255;
}


/*
 * build up the frame's crc
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Sensor::calc_crc(uint8_t byte)
{
    _crc += byte; //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0xFF;
}

/*
 * send the frame's crc at the end of the frame
 * for FrSky SPort protocol (X-receivers)
 *
 * Currently not required as we only send control bytes with single byte payloads that do not require CRC's to be calculated
 */
void AP_Frsky_Sensor::send_crc(void)
{
    send_byte(0xFF - _crc);
    _crc = 0;
}

uint8_t AP_Frsky_Sensor::read_byte()
{
    return _port->read();
}

// Read 32 bit - Note this is different byte order to the telemetry order
uint32_t  AP_Frsky_Sensor::read_uint32()
{
    uint8_t data[4];

    data[0] = _port->read();
    data[1] = _port->read();
    data[2] = _port->read();
    data[3] = _port->read();

    return (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
}

// Read 16 bit - Note this is different byte order to the telemetry order
uint16_t  AP_Frsky_Sensor::read_uint16()
{
    uint8_t data[2];

    data[0] = _port->read();
    data[1] = _port->read();

    return (data[1] << 8) + (data[0]);
}

/*
  send 1 byte and do byte stuffing
*/
void AP_Frsky_Sensor::send_byte(uint8_t byte)
{
    _port->write(byte);
    calc_crc(byte);
}

namespace AP {
    AP_Frsky_Sensor *frsky_sensor() {
        return AP_Frsky_Sensor::get_singleton();
    }
};
