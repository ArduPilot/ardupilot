
#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define SITL_RC_INPUT_CHANNELS 16

#include "AP_HAL_SITL.h"

class HALSITL::RCInput : public AP_HAL::RCInput {
public:
    explicit RCInput(SITL_State *sitlState): _sitlState(sitlState) {}
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override {
        return SITL_RC_INPUT_CHANNELS;
    }
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;

    bool set_overrides(int16_t *overrides, uint8_t len) override;
    bool set_override(uint8_t channel, int16_t override) override;
    void clear_overrides() override;

private:
    SITL_State *_sitlState;

    /* override state */
    uint16_t _override[SITL_RC_INPUT_CHANNELS];
};

#endif

