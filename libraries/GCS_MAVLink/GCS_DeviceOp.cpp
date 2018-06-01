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
  handle device operations over MAVLink
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include "GCS.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
  handle DEVICE_OP_READ message
 */
void GCS_MAVLINK::handle_device_op_read(mavlink_message_t *msg)
{
    mavlink_device_op_read_t packet;
    mavlink_msg_device_op_read_decode(msg, &packet);
    AP_HAL::OwnPtr<AP_HAL::Device> dev = nullptr;
    uint8_t retcode = 0;
    uint8_t data[sizeof(mavlink_device_op_read_reply_t::data)] {};
    
    if (packet.bustype == DEVICE_OP_BUSTYPE_I2C) {
        dev = hal.i2c_mgr->get_device(packet.bus, packet.address);
    } else if (packet.bustype == DEVICE_OP_BUSTYPE_SPI) {
        dev = hal.spi->get_device(packet.busname);
    } else {
        retcode = 1;
        goto fail;
    }
    if (!dev) {
        retcode = 2;
        goto fail;
    }
    if (!dev->get_semaphore()->take(10)) {
        retcode = 3;
        goto fail;        
    }
    if (!dev->read_registers(packet.regstart, data, packet.count)) {
        retcode = 4;
        dev->get_semaphore()->give();
        goto fail;
    }
    dev->get_semaphore()->give();
    mavlink_msg_device_op_read_reply_send(
        chan,
        packet.request_id,
        retcode,
        packet.regstart,
        packet.count,
        data);
    return;

fail:
    mavlink_msg_device_op_read_reply_send(
        chan,
        packet.request_id,
        retcode,
        packet.regstart,
        0,
        nullptr);
}

/*
  handle DEVICE_OP_WRITE message
 */
void GCS_MAVLINK::handle_device_op_write(mavlink_message_t *msg)
{
    mavlink_device_op_write_t packet;
    mavlink_msg_device_op_write_decode(msg, &packet);
    AP_HAL::OwnPtr<AP_HAL::Device> dev = nullptr;
    uint8_t retcode = 0;
    
    if (packet.bustype == DEVICE_OP_BUSTYPE_I2C) {
        dev = hal.i2c_mgr->get_device(packet.bus, packet.address);
    } else if (packet.bustype == DEVICE_OP_BUSTYPE_SPI) {
        dev = hal.spi->get_device(packet.busname);
    } else {
        retcode = 1;
        goto fail;
    }
    if (!dev) {
        retcode = 2;
        goto fail;
    }
    if (!dev->get_semaphore()->take(10)) {
        retcode = 3;
        goto fail;        
    }
    for (uint8_t i=0; i<packet.count; i++) {
        if (!dev->write_register(packet.regstart+i, packet.data[i])) {
            retcode = 4;
            break;
        }
    }
    dev->get_semaphore()->give();

fail:
    mavlink_msg_device_op_write_reply_send(
        chan,
        packet.request_id,
        retcode);
}
