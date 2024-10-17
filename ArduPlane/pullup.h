#pragma once

/*
  support for pullup after NAV_ALTITUDE_WAIT for gliders
 */

#ifndef AP_PLANE_GLIDER_PULLUP_ENABLED
#define AP_PLANE_GLIDER_PULLUP_ENABLED CONFIG_HAL_BOARD == HAL_BOARD_SITL
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED

class GliderPullup
{
public:
    GliderPullup(void);

    void reset(void) {
        stage = Stage::NONE;
    }
    bool in_pullup() const;
    bool verify_pullup(void);
    void stabilize_pullup(void);
    bool pullup_complete(void);
    bool pullup_start(void);

    enum class Stage : uint8_t {
        NONE=0,
        WAIT_AIRSPEED,
        WAIT_PITCH,
        WAIT_LEVEL,
        PUSH_NOSE_DOWN,
    };

    static const struct AP_Param::GroupInfo var_info[];

private:
    Stage stage;
    AP_Int8  enable;
    AP_Float elev_offset; // fraction of full elevator applied during WAIT_AIRSPEED and released during WAIT_PITCH
    AP_Float ng_limit;
    AP_Float airspeed_start;
    AP_Float pitch_start;
    AP_Float ng_jerk_limit;
    AP_Float pitch_dem;
    float ng_demand;
};

#endif // AP_PLANE_GLIDER_PULLUP_ENABLED

