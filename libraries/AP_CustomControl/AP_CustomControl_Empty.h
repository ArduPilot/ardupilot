#pragma once

#include "AP_CustomControl_config.h"

#if AP_CUSTOMCONTROL_EMPTY_ENABLED

#include "AP_CustomControl_Backend.h"

class AP_CustomControl_Empty : public AP_CustomControl_Backend {
public:
    AP_CustomControl_Empty(AP_CustomControl& frontend, float dt);

    void update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    // declare parameters here
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;

    uint32_t statustest_last_t_ms;
};

#endif  // AP_CUSTOMCONTROL_EMPTY_ENABLED
