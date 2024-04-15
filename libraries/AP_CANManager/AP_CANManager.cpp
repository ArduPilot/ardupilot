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
 *   AP_CANManager - board specific configuration for CAN interface
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_CANManager.h"

#if HAL_CANMANAGER_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_KDECAN/AP_KDECAN.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_PiccoloCAN/AP_PiccoloCAN.h>
#include <AP_EFI/AP_EFI_NWPMU.h>
#include <GCS_MAVLink/GCS.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/CANSocketIface.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/CANSocketIface.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#include <AP_HAL_ChibiOS/CANIface.h>
#endif

#include <AP_Common/ExpandingString.h>
#include <AP_Common/sorting.h>

#define LOG_TAG "CANMGR"
#define LOG_BUFFER_SIZE 1024

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_CANManager::var_info[] = {

#if HAL_NUM_CAN_IFACES > 0
    // @Group: P1_
    // @Path: ../AP_CANManager/AP_CANIfaceParams.cpp
    AP_SUBGROUPINFO(_interfaces[0], "P1_", 1, AP_CANManager, AP_CANManager::CANIface_Params),
#endif

#if HAL_NUM_CAN_IFACES > 1
    // @Group: P2_
    // @Path: ../AP_CANManager/AP_CANIfaceParams.cpp
    AP_SUBGROUPINFO(_interfaces[1], "P2_", 2, AP_CANManager, AP_CANManager::CANIface_Params),
#endif

#if HAL_NUM_CAN_IFACES > 2
    // @Group: P3_
    // @Path: ../AP_CANManager/AP_CANIfaceParams.cpp
    AP_SUBGROUPINFO(_interfaces[2], "P3_", 3, AP_CANManager, AP_CANManager::CANIface_Params),
#endif

#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 0
    // @Group: D1_
    // @Path: ../AP_CANManager/AP_CANManager_CANDriver_Params.cpp
    AP_SUBGROUPINFO(_drv_param[0], "D1_", 4, AP_CANManager, AP_CANManager::CANDriver_Params),
#endif

#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 1
    // @Group: D2_
    // @Path: ../AP_CANManager/AP_CANManager_CANDriver_Params.cpp
    AP_SUBGROUPINFO(_drv_param[1], "D2_", 5, AP_CANManager, AP_CANManager::CANDriver_Params),
#endif

#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 2
    // @Group: D3_
    // @Path: ../AP_CANManager/AP_CANManager_CANDriver_Params.cpp
    AP_SUBGROUPINFO(_drv_param[2], "D3_", 6, AP_CANManager, AP_CANManager::CANDriver_Params),
#endif

#if AP_CAN_SLCAN_ENABLED
    // @Group: SLCAN_
    // @Path: ../AP_CANManager/AP_SLCANIface.cpp
    AP_SUBGROUPINFO(_slcan_interface, "SLCAN_", 7, AP_CANManager, SLCAN::CANIface),
#endif

    // @Param: LOGLEVEL
    // @DisplayName: Loglevel
    // @Description: Loglevel for recording initialisation and debug information from CAN Interface
    // @Range: 0 4
    // @Values: 0: Log None, 1: Log Error, 2: Log Warning and below, 3: Log Info and below, 4: Log Everything
    // @User: Advanced
    AP_GROUPINFO("LOGLEVEL", 8, AP_CANManager, _loglevel, AP_CANManager::LOG_NONE),

    AP_GROUPEND
};

AP_CANManager *AP_CANManager::_singleton;

AP_CANManager::AP_CANManager()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_CANManager must be singleton");
    }
    _singleton = this;
}

#if !AP_TEST_DRONECAN_DRIVERS
void AP_CANManager::init()
{
    WITH_SEMAPHORE(_sem);

    // we need to mutate the HAL to install new CAN interfaces
    AP_HAL::HAL& hal_mutable = AP_HAL::get_HAL_mutable();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (AP::sitl() == nullptr) {
        AP_HAL::panic("CANManager: SITL not initialised!");
    }
#endif
    // We only allocate log buffer only when under debug
    if (_loglevel != AP_CANManager::LOG_NONE) {
        _log_buf = new char[LOG_BUFFER_SIZE];
        _log_pos = 0;
    }

#if AP_CAN_SLCAN_ENABLED
    //Reset all SLCAN related params that needs resetting at boot
    _slcan_interface.reset_params();
#endif

    AP_CAN::Protocol drv_type[HAL_MAX_CAN_PROTOCOL_DRIVERS] = {};
    // loop through interfaces and allocate and initialise Iface,
    // Also allocate Driver objects, and add interfaces to them
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        // Get associated Driver to the interface
        uint8_t drv_num = _interfaces[i]._driver_number;
        if (drv_num == 0 || drv_num > HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            continue;
        }
        drv_num--;

        if (hal_mutable.can[i] == nullptr) {
            // So if this interface is not allocated allocate it here,
            // also pass the index of the CANBus
            hal_mutable.can[i] = new HAL_CANIface(i);
        }

        // Initialise the interface we just allocated
        if (hal_mutable.can[i] == nullptr) {
            continue;
        }
        AP_HAL::CANIface* iface = hal_mutable.can[i];

        // Find the driver type that we need to allocate and register this interface with
        drv_type[drv_num] = (AP_CAN::Protocol) _drv_param[drv_num]._driver_type.get();
        bool can_initialised = false;
        // Check if this interface need hooking up to slcan passthrough
        // instead of a driver
#if AP_CAN_SLCAN_ENABLED
        if (_slcan_interface.init_passthrough(i)) {
            // we have slcan bridge setup pass that on as can iface
            can_initialised = hal_mutable.can[i]->init(_interfaces[i]._bitrate, _interfaces[i]._fdbitrate*1000000, AP_HAL::CANIface::NormalMode);
            iface = &_slcan_interface;
        } else {
#else
        if (true) {
#endif
            can_initialised = hal_mutable.can[i]->init(_interfaces[i]._bitrate, _interfaces[i]._fdbitrate*1000000, AP_HAL::CANIface::NormalMode);
        }

        if (!can_initialised) {
            log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "Failed to initialise CAN Interface %d", i+1);
            continue;
        }

        log_text(AP_CANManager::LOG_INFO, LOG_TAG, "CAN Interface %d initialized well", i + 1);

        if (_drivers[drv_num] != nullptr) {
            //We already initialised the driver just add interface and move on
            log_text(AP_CANManager::LOG_INFO, LOG_TAG, "Adding Interface %d to Driver %d", i + 1, drv_num + 1);
            _drivers[drv_num]->add_interface(iface);
            continue;
        }

        if (_num_drivers >= HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            // We are exceeding number of drivers,
            // this can't be happening time to panic
            AP_BoardConfig::config_error("Max number of CAN Drivers exceeded\n\r");
        }

        // Allocate the set type of Driver
#if HAL_ENABLE_DRONECAN_DRIVERS
        if (drv_type[drv_num] == AP_CAN::Protocol::DroneCAN) {
            _drivers[drv_num] = _drv_param[drv_num]._uavcan = new AP_DroneCAN(drv_num);

            if (_drivers[drv_num] == nullptr) {
                AP_BoardConfig::allocation_error("uavcan %d", i + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom((AP_DroneCAN*)_drivers[drv_num], AP_DroneCAN::var_info);
        } else
#endif
#if HAL_PICCOLO_CAN_ENABLE
         if (drv_type[drv_num] == AP_CAN::Protocol::PiccoloCAN) {
            _drivers[drv_num] = _drv_param[drv_num]._piccolocan = new AP_PiccoloCAN;

            if (_drivers[drv_num] == nullptr) {
                AP_BoardConfig::allocation_error("PiccoloCAN %d", drv_num + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom((AP_PiccoloCAN*)_drivers[drv_num], AP_PiccoloCAN::var_info);
        } else
#endif
        {
            continue;
        }

        _num_drivers++;

        // Hook this interface to the selected Driver Type
        _drivers[drv_num]->add_interface(iface);
        log_text(AP_CANManager::LOG_INFO, LOG_TAG, "Adding Interface %d to Driver %d", i + 1, drv_num + 1);

    }

    for (uint8_t drv_num = 0; drv_num < HAL_MAX_CAN_PROTOCOL_DRIVERS; drv_num++) {
        //initialise all the Drivers

        // Cache the driver type, initialized or not, so we can detect that it is in the params at boot via get_driver_type().
        // This allows drivers that are initialized by CANSensor instead of CANManager to know if they should init or not
        _driver_type_cache[drv_num] = drv_type[drv_num];

        if (_drivers[drv_num] == nullptr) {
            continue;
        }
        bool enable_filter = false;
        for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
            if (_interfaces[i]._driver_number == (drv_num+1) &&
                hal_mutable.can[i] != nullptr &&
                hal_mutable.can[i]->get_operating_mode() == AP_HAL::CANIface::FilteredMode) {
                // Don't worry we don't enable Filters for Normal Ifaces under the driver
                // this is just to ensure we enable them for the ones we already decided on
                enable_filter = true;
                break;
            }
        }

        _drivers[drv_num]->init(drv_num, enable_filter);
    }
}
#else
void AP_CANManager::init()
{
    WITH_SEMAPHORE(_sem);
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if ((AP_CAN::Protocol) _drv_param[i]._driver_type.get() == AP_CAN::Protocol::DroneCAN) {
            _drivers[i] = _drv_param[i]._uavcan = new AP_DroneCAN(i);

            if (_drivers[i] == nullptr) {
                AP_BoardConfig::allocation_error("uavcan %d", i + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom((AP_DroneCAN*)_drivers[i], AP_DroneCAN::var_info);
            _drivers[i]->init(i, true);
            _driver_type_cache[i] = (AP_CAN::Protocol) _drv_param[i]._driver_type.get();
        }
    }
}
#endif

/*
  register a new CAN driver
 */
bool AP_CANManager::register_driver(AP_CAN::Protocol dtype, AP_CANDriver *driver)
{
    WITH_SEMAPHORE(_sem);

    // we need to mutate the HAL to install new CAN interfaces
    AP_HAL::HAL& hal_mutable = AP_HAL::get_HAL_mutable();

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        uint8_t drv_num = _interfaces[i]._driver_number;
        if (drv_num == 0 || drv_num > HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            continue;
        }
        // from 1 based to 0 based
        drv_num--;

        if (dtype != (AP_CAN::Protocol)_drv_param[drv_num]._driver_type.get()) {
            continue;
        }
        if (_drivers[drv_num] != nullptr) {
            continue;
        }
        if (_num_drivers >= HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            continue;
        }

        if (hal_mutable.can[i] == nullptr) {
            // if this interface is not allocated allocate it here,
            // also pass the index of the CANBus
            hal_mutable.can[i] = new HAL_CANIface(i);
        }

        // Initialise the interface we just allocated
        if (hal_mutable.can[i] == nullptr) {
            continue;
        }
        AP_HAL::CANIface* iface = hal_mutable.can[i];

        _drivers[drv_num] = driver;
        _drivers[drv_num]->add_interface(iface);
        log_text(AP_CANManager::LOG_INFO, LOG_TAG, "Adding Interface %d to Driver %d", i + 1, drv_num + 1);

        _drivers[drv_num]->init(drv_num, false);
        _driver_type_cache[drv_num] = dtype;

        _num_drivers++;

        return true;
    }
    return false;
}

// register a new auxillary sensor driver for 11 bit address frames
bool AP_CANManager::register_11bit_driver(AP_CAN::Protocol dtype, CANSensor *sensor, uint8_t &driver_index)
{
    WITH_SEMAPHORE(_sem);

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        uint8_t drv_num = _interfaces[i]._driver_number;
        if (drv_num == 0 || drv_num > HAL_MAX_CAN_PROTOCOL_DRIVERS) {
            continue;
        }
        // from 1 based to 0 based
        drv_num--;

        if (dtype != (AP_CAN::Protocol)_drv_param[drv_num]._driver_type_11bit.get()) {
            continue;
        }
        if (_drivers[drv_num] != nullptr &&
            _drivers[drv_num]->add_11bit_driver(sensor)) {
            driver_index = drv_num;
            return true;
        }
    }
    return false;

}

// Method used by CAN related library methods to report status and debug info
// The result of this method can be accessed via ftp get @SYS/can_log.txt
void AP_CANManager::log_text(AP_CANManager::LogLevel loglevel, const char *tag, const char *fmt, ...)
{
    if (_log_buf == nullptr) {
        return;
    }
    if (loglevel > _loglevel) {
        return;
    }

    if ((LOG_BUFFER_SIZE - _log_pos) < (10 + strlen(tag) + strlen(fmt))) {
        // reset log pos
        _log_pos = 0;
    }
    //Tag Log Message
    const char *log_level_tag = "";
    switch (loglevel) {
    case AP_CANManager::LOG_DEBUG :
        log_level_tag = "DEBUG";
        break;

    case AP_CANManager::LOG_INFO :
        log_level_tag = "INFO";
        break;

    case AP_CANManager::LOG_WARNING :
        log_level_tag = "WARN";
        break;

    case AP_CANManager::LOG_ERROR :
        log_level_tag = "ERROR";
        break;

    case AP_CANManager::LOG_NONE:
        return;
    }

    _log_pos += hal.util->snprintf(&_log_buf[_log_pos], LOG_BUFFER_SIZE - _log_pos, "\n%s %s :", log_level_tag, tag);

    va_list arg_list;
    va_start(arg_list, fmt);
    _log_pos += hal.util->vsnprintf(&_log_buf[_log_pos], LOG_BUFFER_SIZE - _log_pos, fmt, arg_list);
    va_end(arg_list);
}

// log retrieve method used by file sys method to report can log
void AP_CANManager::log_retrieve(ExpandingString &str) const
{
    if (_log_buf == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Log buffer not available");
        return;
    }
    str.append(_log_buf, _log_pos);
}

#if HAL_GCS_ENABLED
/*
  handle MAV_CMD_CAN_FORWARD mavlink long command
 */
bool AP_CANManager::handle_can_forward(mavlink_channel_t chan, const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(can_forward.sem);
    const int8_t bus = int8_t(packet.param1)-1;
    if (bus == -1) {
        for (auto can_iface : hal.can) {
            if (can_iface) {
                can_iface->register_frame_callback(nullptr);
            }
        }
        return true;
    }
    if (bus >= HAL_NUM_CAN_IFACES || hal.can[bus] == nullptr) {
        return false;
    }
    if (!hal.can[bus]->register_frame_callback(
            FUNCTOR_BIND_MEMBER(&AP_CANManager::can_frame_callback, void, uint8_t, const AP_HAL::CANFrame &))) {
        return false;
    }
    can_forward.last_callback_enable_ms = AP_HAL::millis();
    can_forward.chan = chan;
    can_forward.system_id = msg.sysid;
    can_forward.component_id = msg.compid;

    // remove registration on other buses, allowing for bus change in the GUI tool
    for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
        if (i != bus && hal.can[i] != nullptr) {
            hal.can[i]->register_frame_callback(nullptr);
        }
    }

    return true;
}

/*
  handle a CAN_FRAME packet
 */
void AP_CANManager::handle_can_frame(const mavlink_message_t &msg)
{
    if (frame_buffer == nullptr) {
        // allocate frame buffer
        WITH_SEMAPHORE(_sem);
        // 20 is good for firmware upload
        uint8_t buffer_size = 20;
        while (frame_buffer == nullptr && buffer_size > 0) {
            // we'd like 20 frames, but will live with less
            frame_buffer = new ObjectBuffer<BufferFrame>(buffer_size);
            if (frame_buffer != nullptr && frame_buffer->get_size() != 0) {
                // register a callback for when frames can't be sent immediately
                hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_CANManager::process_frame_buffer, void));
                break;
            }
            delete frame_buffer;
            frame_buffer = nullptr;
            buffer_size /= 2;
        }
        if (frame_buffer == nullptr) {
            // discard the frames
            return;
        }
    }

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_CAN_FRAME: {
        mavlink_can_frame_t p;
        mavlink_msg_can_frame_decode(&msg, &p);
        if (p.bus >= HAL_NUM_CAN_IFACES || hal.can[p.bus] == nullptr) {
            return;
        }
        struct BufferFrame frame {
            bus : p.bus,
            frame : AP_HAL::CANFrame(p.id, p.data, p.len)
        };
        WITH_SEMAPHORE(_sem);
        frame_buffer->push(frame);
        break;
    }
    case MAVLINK_MSG_ID_CANFD_FRAME: {
        mavlink_canfd_frame_t p;
        mavlink_msg_canfd_frame_decode(&msg, &p);
        if (p.bus >= HAL_NUM_CAN_IFACES || hal.can[p.bus] == nullptr) {
            return;
        }
        struct BufferFrame frame {
            bus : p.bus,
            frame : AP_HAL::CANFrame(p.id, p.data, p.len, true)
        };
        WITH_SEMAPHORE(_sem);
        frame_buffer->push(frame);
        break;
    }
    }
    process_frame_buffer();
}

/*
  process the frame buffer
 */
void AP_CANManager::process_frame_buffer(void)
{
    while (frame_buffer) {
        WITH_SEMAPHORE(_sem);
        struct BufferFrame frame;
        const uint16_t timeout_us = 2000;
        if (!frame_buffer->peek(frame)) {
            // no frames in the queue
            break;
        }
        const int16_t retcode = hal.can[frame.bus]->send(frame.frame,
                                                         AP_HAL::micros64() + timeout_us,
                                                         AP_HAL::CANIface::IsMAVCAN);
        if (retcode == 0) {
            // no space in the CAN output slots, try again later
            break;
        }
        // retcode == 1 means sent, -1 means a frame that can't be
        // sent. Either way we should remove from the queue
        frame_buffer->pop();
    }
}

/*
  handle a CAN_FILTER_MODIFY packet
 */
void AP_CANManager::handle_can_filter_modify(const mavlink_message_t &msg)
{
    mavlink_can_filter_modify_t p;
    mavlink_msg_can_filter_modify_decode(&msg, &p);
    const int8_t bus = int8_t(p.bus)-1;
    if (bus >= HAL_NUM_CAN_IFACES || hal.can[bus] == nullptr) {
        return;
    }
    if (p.num_ids > ARRAY_SIZE(p.ids)) {
        return;
    }
    uint16_t *new_ids = nullptr;
    uint16_t num_new_ids = 0;
    WITH_SEMAPHORE(can_forward.sem);

    // sort the list, so we can bisection search and the array
    // operations below are efficient
    insertion_sort_uint16(p.ids, p.num_ids);
    
    switch (p.operation) {
    case CAN_FILTER_REPLACE: {
        if (p.num_ids == 0) {
            can_forward.num_filter_ids = 0;
            delete[] can_forward.filter_ids;
            can_forward.filter_ids = nullptr;
            return;
        }
        if (p.num_ids == can_forward.num_filter_ids &&
            memcmp(p.ids, can_forward.filter_ids, p.num_ids*sizeof(uint16_t)) == 0) {
            // common case of replacing with identical list
            return;
        }
        new_ids = new uint16_t[p.num_ids];
        if (new_ids != nullptr) {
            num_new_ids = p.num_ids;
            memcpy((void*)new_ids, (const void *)p.ids, p.num_ids*sizeof(uint16_t));
        }
        break;
    }
    case CAN_FILTER_ADD: {
        if (common_list_uint16(can_forward.filter_ids, can_forward.num_filter_ids,
                               p.ids, p.num_ids) == p.num_ids) {
            // nothing changing
            return;
        }
        new_ids = new uint16_t[can_forward.num_filter_ids+p.num_ids];
        if (new_ids == nullptr) {
            return;
        }
        if (can_forward.num_filter_ids != 0) {
            memcpy(new_ids, can_forward.filter_ids, can_forward.num_filter_ids*sizeof(uint16_t));
        }
        memcpy(&new_ids[can_forward.num_filter_ids], p.ids, p.num_ids*sizeof(uint16_t));
        insertion_sort_uint16(new_ids, can_forward.num_filter_ids+p.num_ids);
        num_new_ids = remove_duplicates_uint16(new_ids, can_forward.num_filter_ids+p.num_ids);
        break;
    }
    case CAN_FILTER_REMOVE: {
        if (common_list_uint16(can_forward.filter_ids, can_forward.num_filter_ids,
                               p.ids, p.num_ids) == 0) {
            // nothing changing
            return;
        }
        can_forward.num_filter_ids = remove_list_uint16(can_forward.filter_ids, can_forward.num_filter_ids,
                                                        p.ids, p.num_ids);
        if (can_forward.num_filter_ids == 0) {
            delete[] can_forward.filter_ids;
            can_forward.filter_ids = nullptr;
        }
        break;
    }
    }
    if (new_ids != nullptr) {
        // handle common case of no change
        if (num_new_ids == can_forward.num_filter_ids &&
            memcmp(new_ids, can_forward.filter_ids, num_new_ids*sizeof(uint16_t)) == 0) {
            delete[] new_ids;
        } else {
            // put the new list in place
            delete[] can_forward.filter_ids;
            can_forward.filter_ids = new_ids;
            can_forward.num_filter_ids = num_new_ids;
        }
    }
}

/*
  handler for CAN frames from the registered callback, sending frames
  out as CAN_FRAME or CANFD_FRAME messages
 */
void AP_CANManager::can_frame_callback(uint8_t bus, const AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(can_forward.sem);
    if (can_forward.frame_counter++ == 100) {
        // check every 100 frames for disabling CAN_FRAME send
        // we stop sending after 5s if the client stops
        // sending MAV_CMD_CAN_FORWARD requests
        if (AP_HAL::millis() - can_forward.last_callback_enable_ms > 5000) {
            hal.can[bus]->register_frame_callback(nullptr);
            return;
        }
        can_forward.frame_counter = 0;
    }
    WITH_SEMAPHORE(comm_chan_lock(can_forward.chan));
    if (can_forward.filter_ids != nullptr) {
        // work out ID of this frame
        uint16_t id = 0;
        if ((frame.id&0xff) != 0) {
            // not anonymous
            if (frame.id & 0x80) {
                // service message
                id = uint8_t(frame.id>>16);
            } else {
                // message frame
                id = uint16_t(frame.id>>8);
            }
        }
        if (!bisect_search_uint16(can_forward.filter_ids, can_forward.num_filter_ids, id)) {
            return;
        }
    }
    const uint8_t data_len = AP_HAL::CANFrame::dlcToDataLength(frame.dlc);
    if (frame.isCanFDFrame()) {
        if (HAVE_PAYLOAD_SPACE(can_forward.chan, CANFD_FRAME)) {
            mavlink_msg_canfd_frame_send(can_forward.chan, can_forward.system_id, can_forward.component_id,
                                         bus, data_len, frame.id, const_cast<uint8_t*>(frame.data));
        }
    } else {
        if (HAVE_PAYLOAD_SPACE(can_forward.chan, CAN_FRAME)) {
            mavlink_msg_can_frame_send(can_forward.chan, can_forward.system_id, can_forward.component_id,
                                       bus, data_len, frame.id, const_cast<uint8_t*>(frame.data));
        }
    }
}
#endif // HAL_GCS_ENABLED

AP_CANManager& AP::can()
{
    return *AP_CANManager::get_singleton();
}

#endif

