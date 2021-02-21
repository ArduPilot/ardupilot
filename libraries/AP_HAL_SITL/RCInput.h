
#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
#define SITL_RC_INPUT_CHANNELS 16

#include "AP_HAL_SITL.h"

class HALSITL::RCInput : public AP_HAL::RCInput {
public:
    explicit RCInput(SITL_State *sitlState): _sitlState(sitlState) {}
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;

    const char *protocol() const override { return "SITL"; }

private:
    SITL_State *_sitlState;
    bool using_rc_protocol;
};

#endif

