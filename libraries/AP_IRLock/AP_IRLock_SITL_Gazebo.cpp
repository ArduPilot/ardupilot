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
 * AP_IRLock_SITL.cpp
 *
 *  Created on: June 09, 2016
 *      Author: Ian Chen
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_IRLock_SITL_Gazebo.h"
#include <SITL/SITL.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>

extern const AP_HAL::HAL& hal;

AP_IRLock_SITL_Gazebo::AP_IRLock_SITL_Gazebo() :
    _last_timestamp(0),
    sock(true)
{}

void AP_IRLock_SITL_Gazebo::init(int8_t bus)
{
    SITL::SITL *sitl = AP::sitl();
    // try to bind to a specific port so that if we restart ArduPilot
    // Gazebo keeps sending us packets. Not strictly necessary but
    // useful for debugging
    sock.bind("127.0.0.1", sitl->irlock_port);

    sock.reuseaddress();
    sock.set_blocking(false);

    hal.console->printf("AP_IRLock_SITL::init()\n");

    _flags.healthy = true;
}

// retrieve latest sensor data - returns true if new data is available
bool AP_IRLock_SITL_Gazebo::update()
{
    // return immediately if not healthy
    if (!_flags.healthy) {
        return false;
    }

    // receive packet from Gazebo IRLock plugin
    irlock_packet pkt;
    const int wait_ms = 0;
    ssize_t s = sock.recv(&pkt, sizeof(irlock_packet), wait_ms);

    bool new_data = false;

    if (s == sizeof(irlock_packet) && pkt.timestamp > _last_timestamp) {
        // fprintf(stderr, "     posx %f posy %f sizex %f sizey %f\n", pkt.pos_x, pkt.pos_y, pkt.size_x, pkt.size_y);
        _target_info.timestamp = pkt.timestamp;
        _target_info.pos_x = pkt.pos_x;
        _target_info.pos_y = pkt.pos_y;
        _target_info.size_x = pkt.size_x;
        _target_info.size_y = pkt.size_y;
        _last_timestamp = pkt.timestamp;
        _last_update_ms = _last_timestamp;
        new_data = true;
    }

    // return true if new data found
    return new_data;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
