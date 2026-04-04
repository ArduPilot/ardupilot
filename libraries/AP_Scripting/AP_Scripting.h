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

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include <GCS_MAVLink/GCS_config.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_Scripting_CANSensor.h"
#include <AP_Networking/AP_Networking_Config.h>

#ifndef SCRIPTING_MAX_NUM_I2C_DEVICE
  #define SCRIPTING_MAX_NUM_I2C_DEVICE 4
#endif

#define SCRIPTING_MAX_NUM_PWM_SOURCE 4

#if AP_NETWORKING_ENABLED
#ifndef SCRIPTING_MAX_NUM_NET_SOCKET
#define SCRIPTING_MAX_NUM_NET_SOCKET 50
#endif
class SocketAPM;
#endif

#if AP_SCRIPTING_SERIALDEVICE_ENABLED
#include "AP_Scripting_SerialDevice.h"
#endif

class AP_Scripting
{
public:
    AP_Scripting();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Scripting);

    void init(void);

#if AP_SCRIPTING_SERIALDEVICE_ENABLED
    void init_serialdevice_ports(void);
#endif

    void update();

    bool enabled(void) const { return _enable != 0; };
    bool should_run(void) const { return enabled() && !_stop; }

#if HAL_GCS_ENABLED
    void handle_message(const mavlink_message_t &msg, const mavlink_channel_t chan);

    // Check if command ID is blocked
    bool is_handling_command(uint16_t cmd_id);
#endif

    static AP_Scripting * get_singleton(void) { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

#if HAL_GCS_ENABLED
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);
#endif

    void handle_mission_command(const class AP_Mission::Mission_Command& cmd);

    bool arming_checks(size_t buflen, char *buffer) const;
    
    void restart_all(void);
    void stop(void) { _stop = true; }

   // User parameters for inputs into scripts 
   AP_Float _user[6];

    enum class SCR_DIR {
        ROMFS = 1 << 0,
        SCRIPTS = 1 << 1,
    };
    uint16_t get_disabled_dir() { return uint16_t(_dir_disable.get());}

    // the number of and storage for i2c devices
    uint8_t num_i2c_devices;
    AP_HAL::I2CDevice *_i2c_dev[SCRIPTING_MAX_NUM_I2C_DEVICE];

#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    // Scripting CAN sensor
    ScriptingCANSensor *_CAN_dev;
    ScriptingCANSensor *_CAN_dev2;
#endif

#if AP_MISSION_ENABLED
    // mission item buffer
    static const int mission_cmd_queue_size = 5;
    struct scripting_mission_cmd {
        uint16_t p1;
        float content_p1;
        float content_p2;
        float content_p3;
        uint32_t time_ms;
    };
    ObjectBuffer<struct scripting_mission_cmd> * mission_data;
#endif

    // PWMSource storage
    uint8_t num_pwm_source;
    AP_HAL::PWMSource *_pwm_source[SCRIPTING_MAX_NUM_PWM_SOURCE];
    int get_current_env_ref() { return current_env_ref; }
    void set_current_env_ref(int ref) { current_env_ref = ref; }

#if AP_NETWORKING_ENABLED
    // SocketAPM storage
    SocketAPM *_net_sockets[SCRIPTING_MAX_NUM_NET_SOCKET];
#endif

    struct mavlink_msg {
        mavlink_message_t msg;
        mavlink_channel_t chan;
        uint32_t timestamp_ms;
    };

    struct mavlink {
        ObjectBuffer<struct mavlink_msg> *rx_buffer;
        uint32_t *accept_msg_ids;
        uint16_t accept_msg_ids_size;
        HAL_Semaphore sem;
    } mavlink_data;

    struct command_block_list {
        uint16_t id;
        command_block_list *next;
    };
    command_block_list *mavlink_command_block_list;
    HAL_Semaphore mavlink_command_block_list_sem;

    #if AP_SCRIPTING_SERIALDEVICE_ENABLED
        AP_Scripting_SerialDevice _serialdevice;
    #endif

    enum class DebugOption : uint8_t {
        NO_SCRIPTS_TO_RUN = 1U << 0,
        RUNTIME_MSG = 1U << 1,
        SUPPRESS_SCRIPT_LOG = 1U << 2,
        LOG_RUNTIME = 1U << 3,
        DISABLE_PRE_ARM = 1U << 4,
        SAVE_CHECKSUM = 1U << 5,
        DISABLE_HEAP_EXPANSION = 1U << 6,
    };

private:

    void thread(void); // main script execution thread

    // Check if DEBUG_OPTS bit has been set to save current checksum values to params
    void save_checksum();

    // Mask down to 23 bits for comparison with parameters, this the length of the a float mantissa, to deal with the float transport of parameters over MAVLink
    // The full range of uint32 integers cannot be represented by a float.
    const uint32_t checksum_param_mask = 0x007FFFFF;

    enum class ThreadPriority : uint8_t {
        NORMAL = 0,
        IO = 1,
        STORAGE = 2,
        UART = 3,
        I2C = 4,
        SPI = 5,
        TIMER = 6,
        MAIN = 7,
        BOOST = 8
    };

    AP_Int8 _enable;
    AP_Int32 _script_vm_exec_count;
    AP_Int32 _script_heap_size;
    AP_Int8 _debug_options;
    AP_Int16 _dir_disable;
    AP_Int32 _required_loaded_checksum;
    AP_Int32 _required_running_checksum;

    AP_Enum<ThreadPriority> _thd_priority;

    bool option_is_set(DebugOption option) const {
        return (uint8_t(_debug_options.get()) & uint8_t(option)) != 0;
    }

    void option_clear(DebugOption option) {
        _debug_options.set_and_save(_debug_options.get() & ~uint8_t(option));
    }

    bool _thread_failed; // thread allocation failed
    bool _init_failed;  // true if memory allocation failed
    bool _restart; // true if scripts should be restarted
    bool _stop; // true if scripts should be stopped

    static AP_Scripting *_singleton;
    int current_env_ref;
};

namespace AP {
    AP_Scripting * scripting(void);
};

#endif // AP_SCRIPTING_ENABLED
