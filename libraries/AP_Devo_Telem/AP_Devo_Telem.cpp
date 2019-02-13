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
   DEVO Telemetry library
*/

#define DEVOM_SYNC_BYTE        0xAA


#define AP_SERIALMANAGER_DEVO_TELEM_BAUD        38400
#define AP_SERIALMANAGER_DEVO_BUFSIZE_RX        0
#define AP_SERIALMANAGER_DEVO_BUFSIZE_TX        32

#include "AP_Devo_Telem.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

extern const AP_HAL::HAL& hal;


//constructor
AP_DEVO_Telem::AP_DEVO_Telem()
{
    devoPacket.header = DEVOM_SYNC_BYTE;
}

void AP_DEVO_Telem::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Devo_Telem, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_DEVO_TELEM_BAUD, AP_SERIALMANAGER_DEVO_BUFSIZE_RX, AP_SERIALMANAGER_DEVO_BUFSIZE_TX);

        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_DEVO_Telem::tick, void));
    }
}


uint32_t AP_DEVO_Telem::gpsDdToDmsFormat(float ddm)
{
    int32_t deg = (int32_t)ddm;
    float mm = (ddm - deg) * 60.0f;

    mm = ((float)deg * 100.0f + mm) /100.0;

    if ((mm < -180.0f) || (mm > 180.0f)) {
        mm = 0.0f;
    }

    return mm * 1.0e7;
}


/*
  send_frames - sends updates down telemetry link
  should be called at 1hz
*/

#define DEVO_SPEED_FACTOR 0.0194384f

void AP_DEVO_Telem::send_frames(uint8_t control_mode)
{
    // return immediately if not initialised
    if (_port == nullptr) {
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();
    const AP_GPS &gps = AP::gps();
    Location loc;

    if (_ahrs.get_position(loc)) {
        devoPacket.lat = gpsDdToDmsFormat(loc.lat);
        devoPacket.lon = gpsDdToDmsFormat(loc.lng);
        devoPacket.speed = (int16_t)(gps.ground_speed() * DEVO_SPEED_FACTOR * 100.0f);  // * 100 for cm

        /*
          Note that this isn't actually barometric altitude, it is the
          inertial nav estimate of altitude above home.
        */
        float alt;
        _ahrs.get_relative_position_D_home(alt);
        devoPacket.alt   = alt * -100.0f; // coordinates was in NED, so it needs to change sign. Protocol requires in cm!
    } else {
        devoPacket.lat = 0;
        devoPacket.lon = 0;
        devoPacket.speed = 0;
        devoPacket.alt=0;
    }



    devoPacket.volt = roundf(AP::battery().voltage() * 10.0f);
    devoPacket.temp = control_mode; // Send mode as temperature

    devoPacket.checksum8 = 0; // Init Checksum with zero Byte

    uint8_t *b = (uint8_t *)&devoPacket;
    for (uint8_t i = sizeof(devoPacket)-1; i !=0; i--) {
        _port->write(b, 1);
        devoPacket.checksum8 += *b++; // Add Checksum
    }
    _port->write(&devoPacket.checksum8, 1);
}

void AP_DEVO_Telem::tick(void)
{
    uint32_t now = AP_HAL::millis();

    if (now - _last_frame_ms > 1000) {
        _last_frame_ms = now;
        send_frames(_control_mode);
    }
}
