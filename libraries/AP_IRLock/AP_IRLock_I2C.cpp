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
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_IRLock_I2C.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define IRLOCK_I2C_ADDRESS		0x54

#define IRLOCK_SYNC			0xAA55
#define IRLOCK_RESYNC		0x5500
#define IRLOCK_ADJUST		0xAA

void AP_IRLock_I2C::init()
{
    AP_HAL::OwnPtr<AP_HAL::Device> tdev = hal.i2c_mgr->get_device(1, IRLOCK_I2C_ADDRESS);
    if (!tdev || !tdev->get_semaphore()->take(0)) {
        return;
    }

    // get initial frame
    read_frame();

    tdev->get_semaphore()->give();
    
    if (_flags.healthy) {
        // read at 50Hz
        printf("Found IRLock on I2C\n");
        dev = std::move(tdev);

        sem = hal.util->new_semaphore();        

        dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&AP_IRLock_I2C::read_frame, bool));
    }
}

/* 
   synchronise with frame start
*/
bool AP_IRLock_I2C::sync_frame_start(void)
{
	uint16_t sync_word;
    if (!dev->transfer(nullptr, 0, (uint8_t *)&sync_word, 2)) {
		return false;
	}
    if (sync_word == IRLOCK_SYNC) {
        return true;
    }
    if (sync_word != IRLOCK_RESYNC) {
        return false;
    }
    uint8_t sync_byte;
    if (!dev->transfer(nullptr, 0, &sync_byte, 1)) {
        return false;
    }
    if (sync_byte == IRLOCK_ADJUST) {
        return true;
    }
	return false;
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
bool AP_IRLock_I2C::read_frame(void)
{
	if (!sync_frame_start()) {
		return false;
	}

    struct frame irframe;
    if (!dev->transfer(nullptr, 0, (uint8_t*)&irframe, sizeof(irframe))) {
        return false;
    }

	/* check crc */
	if (irframe.signature + irframe.pixel_x + irframe.pixel_y + irframe.pixel_size_x + irframe.pixel_size_y !=
        irframe.checksum) {
        return false;
	}

	int16_t corner1_pix_x = irframe.pixel_x - irframe.pixel_size_x/2;
    int16_t corner1_pix_y = irframe.pixel_y - irframe.pixel_size_y/2;
    int16_t corner2_pix_x = irframe.pixel_x + irframe.pixel_size_x/2;
    int16_t corner2_pix_y = irframe.pixel_y + irframe.pixel_size_y/2;
    
    float corner1_pos_x, corner1_pos_y, corner2_pos_x, corner2_pos_y;
    pixel_to_1M_plane(corner1_pix_x, corner1_pix_y, corner1_pos_x, corner1_pos_y);
    pixel_to_1M_plane(corner2_pix_x, corner2_pix_y, corner2_pos_x, corner2_pos_y);

    if (sem->take(0)) {
        /* convert to angles */
        _target_info.timestamp = AP_HAL::millis();
        _target_info.pos_x = 0.5f*(corner1_pos_x+corner2_pos_x);
        _target_info.pos_y = 0.5f*(corner1_pos_y+corner2_pos_y);
        _target_info.size_x = corner2_pos_x-corner1_pos_x;
        _target_info.size_y = corner2_pos_y-corner1_pos_y;
        sem->give();
    }
    
    return true;
}

// retrieve latest sensor data - returns true if new data is available
bool AP_IRLock_I2C::update()
{
    bool new_data = false;
    if (!sem) {
        return false;
    }
    if (sem->take(0)) {
        if (_last_update_ms != _target_info.timestamp) {
            new_data = true;
        }
        _last_update_ms = _target_info.timestamp;
        _flags.healthy = (AP_HAL::millis() - _last_update_ms < 100);
        sem->give();
    }
    // return true if new data found
    return new_data;
}
