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
 * AP_IRLock_I2C.cpp
 *
 * Based on AP_IRLock_PX4 by MLandes
 *
 * See: http://irlock.com/pages/serial-communication-protocol
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_IRLock_I2C.h"
#include <stdio.h>
#include <utility>
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL& hal;

#define IRLOCK_I2C_ADDRESS      0x54

#define IRLOCK_SYNC         0xAA55AA55

void AP_IRLock_I2C::init(int8_t bus)
{
    if (bus < 0) {
        // default to i2c external bus
        bus = 1;
    }
    dev = std::move(hal.i2c_mgr->get_device(bus, IRLOCK_I2C_ADDRESS));
    if (!dev) {
        return;
    }

    // read at 50Hz
    printf("Starting IRLock on I2C\n");

    dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&AP_IRLock_I2C::read_frames, void));
}

/*
   synchronise with frame start. We expect 0xAA55AA55 at the start of
   a frame
*/
bool AP_IRLock_I2C::sync_frame_start(void)
{
    uint32_t sync_word;
    if (!dev->transfer(nullptr, 0, (uint8_t *)&sync_word, 4)) {
        return false;
    }

    // record sensor successfully responded to I2C request
    _last_read_ms = AP_HAL::millis();

    uint8_t count=40;
    while (count-- && sync_word != IRLOCK_SYNC && sync_word != 0) {
        uint8_t sync_byte;
        if (!dev->transfer(nullptr, 0, &sync_byte, 1)) {
            return false;
        }
        if (sync_byte == 0) {
            break;
        }
        sync_word = (sync_word>>8) | (uint32_t(sync_byte)<<24);
    }
    return sync_word == IRLOCK_SYNC;
}

/*
  converts IRLOCK pixels to a position on a normal plane 1m in front of the lens
  based on a characterization of IR-LOCK with the standard lens, focused such that 2.38mm of threads are exposed
 */
void AP_IRLock_I2C::pixel_to_1M_plane(float pix_x, float pix_y, float &ret_x, float &ret_y)
{
    ret_x = (-0.00293875727162397f*pix_x + 0.470201163459835f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
                                                                4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
    ret_y = (-0.003056843086277f*pix_y + 0.3056843086277f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
                                                            4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
}

/*
  read a frame from sensor
*/
bool AP_IRLock_I2C::read_block(struct frame &irframe)
{
    if (!dev->transfer(nullptr, 0, (uint8_t*)&irframe, sizeof(irframe))) {
        return false;
    }

    // record sensor successfully responded to I2C request
    _last_read_ms = AP_HAL::millis();

    /* check crc */
    uint32_t crc = irframe.signature + irframe.pixel_x + irframe.pixel_y + irframe.pixel_size_x + irframe.pixel_size_y;
    if (crc != irframe.checksum) {
        // printf("bad crc 0x%04x 0x%04x\n", crc, irframe.checksum);
        return false;
    }
    return true;
}

void AP_IRLock_I2C::read_frames(void)
{
    if (!sync_frame_start()) {
        return;
    }
    struct frame irframe;
    
    if (!read_block(irframe)) {
        return;
    }

    int16_t corner1_pix_x = irframe.pixel_x - irframe.pixel_size_x/2;
    int16_t corner1_pix_y = irframe.pixel_y - irframe.pixel_size_y/2;
    int16_t corner2_pix_x = irframe.pixel_x + irframe.pixel_size_x/2;
    int16_t corner2_pix_y = irframe.pixel_y + irframe.pixel_size_y/2;

    float corner1_pos_x, corner1_pos_y, corner2_pos_x, corner2_pos_y;
    pixel_to_1M_plane(corner1_pix_x, corner1_pix_y, corner1_pos_x, corner1_pos_y);
    pixel_to_1M_plane(corner2_pix_x, corner2_pix_y, corner2_pos_x, corner2_pos_y);

    {
        WITH_SEMAPHORE(sem);

        /* convert to angles */
        _target_info.timestamp = AP_HAL::millis();
        _target_info.pos_x = 0.5f*(corner1_pos_x+corner2_pos_x);
        _target_info.pos_y = 0.5f*(corner1_pos_y+corner2_pos_y);
        _target_info.pos_z = 1.0f;
    }

#if 0
    // debugging
    static uint32_t lastt;
    if (_target_info.timestamp - lastt > 2000) {
        lastt = _target_info.timestamp;
        printf("pos_x:%.5f pos_y:%.5f size_x:%.6f size_y:%.5f\n", 
               _target_info.pos_x, _target_info.pos_y,
               (corner2_pos_x-corner1_pos_x), (corner2_pos_y-corner1_pos_y));
    }
#endif
}

// retrieve latest sensor data - returns true if new data is available
bool AP_IRLock_I2C::update()
{
    bool new_data = false;
    if (!dev) {
        return false;
    }
    WITH_SEMAPHORE(sem);

    if (_last_update_ms != _target_info.timestamp) {
        new_data = true;
    }
    _last_update_ms = _target_info.timestamp;
    _flags.healthy = (AP_HAL::millis() - _last_read_ms < 100);

    // return true if new data found
    return new_data;
}
