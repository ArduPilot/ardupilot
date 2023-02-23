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

#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_Scripting_CANSensor.h"
#include <AP_RTC/JitterCorrection.h>

#ifndef SCRIPTING_MAX_NUM_I2C_DEVICE
  #define SCRIPTING_MAX_NUM_I2C_DEVICE 4
#endif

class AP_Scripting
{
public:
    AP_Scripting();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Scripting);

    void init(void);

    bool enabled(void) const { return _enable != 0; };
    bool should_run(void) const { return enabled() && !_stop; }

    static AP_Scripting * get_singleton(void) { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);

    void handle_mission_command(const class AP_Mission::Mission_Command& cmd);

    bool arming_checks(size_t buflen, char *buffer) const;
    
    void restart_all(void);

   // User parameters for inputs into scripts 
   AP_Float _user[6];

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

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // Scripting CAN sensor
    ScriptingCANSensor *_CAN_dev;
    ScriptingCANSensor *_CAN_dev2;
#endif

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


    // linked list of named values and type
    class NamedValue {
    public:
        enum TYPE {
            TYPE_NIL = 0,
            TYPE_INT = 1,
            TYPE_FLOAT = 2,
        } type;
        NamedValue(const char *_name, enum TYPE _type, NamedValue *_next) :
            type(_type),
            next(_next)
        {
            strncpy_noterm(name, _name, sizeof(name));
        }
        char name[10];
        union Value {
            int32_t i;
            float f;
        } value;

        NamedValue *next;
        uint32_t timestamp_ms;
        uint8_t sysid;
        uint8_t compid;
        JitterCorrection jitter;

        // add value to linked list, return true if added or already exists, false if out of memory
        static bool register_name(const char *name) {
            WITH_SEMAPHORE(sem);
            NamedValue *nv = named_values;
            if (strlen(name) > 10) {
                return false;
            }
            while (nv) {
                if (strncmp(nv->name, name, 10) == 0) {
                    return true;
                }
                nv = nv->next;
            }
            nv = new NamedValue(name, TYPE_NIL, named_values);
            if (nv == nullptr) {
                return false;
            }
            nv->next = named_values;
            named_values = nv;
            return true;
        }
        
        static NamedValue* get(const char* _name) {
            WITH_SEMAPHORE(sem);
            NamedValue *nv = named_values;
            if (strlen(_name) > 10) {
                return nullptr;
            }
            while (nv) {
                if (strncmp(nv->name, _name, 10) == 0) {
                    return nv;
                }
                nv = nv->next;
            }
            return nullptr;
        }

        static HAL_Semaphore sem;
    private:
        static NamedValue *named_values;
    };

    void set_named_value(const char *name, int32_t value, uint32_t timestamp_ms, uint8_t sysid, uint8_t compid) {
        NamedValue *nv = NamedValue::get(name);
        if (nv == nullptr) {
            return;
        }
        WITH_SEMAPHORE(NamedValue::sem);
        nv->type = NamedValue::TYPE_INT;
        nv->value.i = value;
        nv->timestamp_ms = nv->jitter.correct_offboard_timestamp_msec(timestamp_ms, AP_HAL::millis());
        nv->sysid = sysid;
        nv->compid = compid;
    }

    void set_named_value(const char *name, float value, uint32_t timestamp_ms, uint8_t sysid, uint8_t compid) {
        NamedValue *nv = NamedValue::get(name);
        if (nv == nullptr) {
            return;
        }
        WITH_SEMAPHORE(NamedValue::sem);
        nv->type = NamedValue::TYPE_FLOAT;
        nv->value.f = value;
        nv->timestamp_ms = nv->jitter.correct_offboard_timestamp_msec(timestamp_ms, AP_HAL::millis());
        nv->sysid = sysid;
        nv->compid = compid;
    }

private:

    bool repl_start(void);
    void repl_stop(void);

    void load_script(const char *filename); // load a script from a file

    void thread(void); // main script execution thread

    AP_Int8 _enable;
    AP_Int32 _script_vm_exec_count;
    AP_Int32 _script_heap_size;
    AP_Int8 _debug_options;
    AP_Int16 _dir_disable;

    bool _thread_failed; // thread allocation failed
    bool _init_failed;  // true if memory allocation failed
    bool _restart; // true if scripts should be restarted
    bool _stop; // true if scripts should be stopped

    static AP_Scripting *_singleton;

};

namespace AP {
    AP_Scripting * scripting(void);
};

#endif // AP_SCRIPTING_ENABLED
