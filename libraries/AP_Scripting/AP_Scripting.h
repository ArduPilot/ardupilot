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

#ifdef ENABLE_SCRIPTING

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/I2CDevice.h>

#ifndef SCRIPTING_MAX_NUM_I2C_DEVICE
  #define SCRIPTING_MAX_NUM_I2C_DEVICE 4
#endif

class AP_Scripting
{
public:
    AP_Scripting();

    /* Do not allow copies */
    AP_Scripting(const AP_Scripting &other) = delete;
    AP_Scripting &operator=(const AP_Scripting&) = delete;

    void init(void);
    bool init_failed(void) const { return _init_failed; }

    bool enabled(void) const { return _enable != 0; };

    static AP_Scripting * get_singleton(void) { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);

    void handle_mission_command(const AP_Mission::Mission_Command& cmd);

   // User parameters for inputs into scripts 
   AP_Float _user[4]; 

    struct terminal_s {
        int output_fd;
        off_t input_offset;
        bool session;
    } terminal;

    enum class SCR_DIR {
        ROMFS = 1 << 0,
        SCRIPTS = 1 << 1,
    };
    uint16_t get_disabled_dir() { return uint16_t(_dir_disable.get());}

    // the number of and storage for i2c devices
    uint8_t num_i2c_devices;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> *_i2c_dev[SCRIPTING_MAX_NUM_I2C_DEVICE];

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

private:

    bool repl_start(void);
    void repl_stop(void);

    void load_script(const char *filename); // load a script from a file

    void thread(void); // main script execution thread

    AP_Int8 _enable;
    AP_Int32 _script_vm_exec_count;
    AP_Int32 _script_heap_size;
    AP_Int8 _debug_level;
    AP_Int16 _dir_disable;

    bool _init_failed;  // true if memory allocation failed

    static AP_Scripting *_singleton;

};

namespace AP {
    AP_Scripting * scripting(void);
};

#endif // ENABLE_SCRIPTING
