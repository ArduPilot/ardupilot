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

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define FRSKY_FLVSS_MAX_CELLS   6
#define FRSKY_MAX_FLVSS_INSTANCES 2

#define START_STOP_SPORT            0x7E
#define BYTESTUFF_SPORT             0x7D
#define SPORT_DATA_FRAME            0x10

class AP_Frsky_Sensor {
public:
    AP_Frsky_Sensor();

    ~AP_Frsky_Sensor();

    /* Do not allow copies */
    AP_Frsky_Sensor(const AP_Frsky_Sensor &other) = delete;
    AP_Frsky_Sensor &operator=(const AP_Frsky_Sensor&) = delete;

    // init - perform required initialisation
    bool init();

    // add statustext message to FrSky lib message queue
    //void queue_message(MAV_SEVERITY severity, const char *text);

    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    uint32_t sensor_status_flags() const;

    static AP_Frsky_Sensor *get_singleton(void) {
        return singleton;
    }

    // get next telemetry data for external consumers of SPort data
    //static bool get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data);

    // tick - main call to send updates to transmitter (called by scheduler at 1kHz)
    void loop(void);
    float get_SPort_voltage(uint16_t id);
    uint32_t get_SPort_read_time(uint16_t id);
    uint8_t get_SPort_num_cells(uint16_t id);
    uint16_t get_SPort_cells(uint16_t id, uint8_t cell_num);

private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    uint16_t _crc;

    //uint8_t _paramID;

    bool _receiving = false;

    //void send_SPort_Passthrough(void);
    // main transmission function when protocol is FrSky SPort
    void send_SPort(void);
    // main transmission function when protocol is FrSky D
    //void send_D(void);
    // methods related to the nuts-and-bolts of sending data
    void calc_crc(uint8_t byte);
    void send_crc(void);
    void send_byte(uint8_t value);
    void send_uint16(uint16_t id, uint16_t data);

    uint8_t read_byte();
    uint16_t  read_uint16();
    uint32_t  read_uint32();
    void read_SPort_Sensors(uint32_t now);
    uint8_t getFLVSS_inst(uint16_t id);

    struct FLVSS_State {
        uint16_t    id;
        uint8_t     num_cells;
        uint16_t cell[FRSKY_FLVSS_MAX_CELLS];
        float       voltage;
        uint32_t    last_reading_ms;
    } _FLVSS_inst[FRSKY_MAX_FLVSS_INSTANCES];

    static AP_Frsky_Sensor *singleton;

    uint8_t _num_sensors_FLVSS = 0;
    uint8_t _num_devices_found = 0;
    uint8_t _device_discovered_loop_count=0;
    uint8_t _deviceID_loop_count=0;
    uint8_t _previous_byte;
    uint8_t _discovered_ids_list[28];
    uint8_t _loop_counter=0;
};

namespace AP {
    AP_Frsky_Sensor *frsky_sensor();
};
