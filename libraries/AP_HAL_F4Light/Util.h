
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_F4Light_Namespace.h"

#include <AP_Param/AP_Param.h>
#include <AP_Param_Helper/AP_Param_Helper.h>

extern "C" {
 void get_board_serial(uint8_t *serialid);
};

class F4Light::Util : public AP_HAL::Util {
public:
    Util(): gps_shift(0) {}

    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; } // shell in FC? you must be kidding!

    void set_soft_armed(const bool b) { 
        if(soft_armed != b){ 
            soft_armed = b;
            Scheduler::arming_state_changed(b); 
        }
    }
    
    inline bool get_soft_armed() { return soft_armed; }

    uint32_t available_memory(void) override
    {
        return 128*1024;
    }
    
    bool get_system_id(char buf[40])  override {
        uint8_t serialid[12];
        memset(serialid, 0, sizeof(serialid));
        get_board_serial(serialid);

        const char *board_type = BOARD_OWN_NAME;

        // this format is chosen to match the human_readable_serial()
        // function in auth.c
        snprintf(buf, 40, "%s %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
             board_type,
             (unsigned)serialid[0], (unsigned)serialid[1], (unsigned)serialid[2], (unsigned)serialid[3],
             (unsigned)serialid[4], (unsigned)serialid[5], (unsigned)serialid[6], (unsigned)serialid[7],
             (unsigned)serialid[8], (unsigned)serialid[9], (unsigned)serialid[10],(unsigned)serialid[11]);
        return true;
    }
    
    // create a new semaphore
    Semaphore *new_semaphore(void)  override { return new F4Light::Semaphore; } 

    void *malloc_type(size_t size, Memory_Type mem_type) override;
    void free_type(void *ptr, size_t size, Memory_Type mem_type) override;
    
private:
    uint64_t gps_shift; // shift from board time to real time
};


