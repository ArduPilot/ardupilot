#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_SITL_Namespace.h"
#include "AP_HAL_SITL.h"
#include "Semaphores.h"

class HALSITL::Util : public AP_HAL::Util {
public:
    Util(SITL_State *_sitlState) :
        sitlState(_sitlState) {}
    
    bool run_debug_shell(AP_HAL::BetterStream *stream) override {
        return false;
    }

    /**
       how much free memory do we have in bytes. 
     */
    uint32_t available_memory(void) override {
        // SITL is assumed to always have plenty of memory. Return 128k for now
        return 0x20000;
    }

    // get path to custom defaults file for AP_Param
    const char* get_custom_defaults_file() const override {
        return sitlState->defaults_path;
    }

    uint64_t get_hw_rtc() const override;

    bool get_system_id(char buf[40]) override;
    bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) override;
    
private:
    SITL_State *sitlState;
};
