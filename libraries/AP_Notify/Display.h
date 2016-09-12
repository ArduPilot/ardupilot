/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "NotifyDevice.h"

#define ROW(Y) ((Y * 11) + 2)
#define COLUMN(X) ((X * 7) + 1)

class Display: public NotifyDevice {
public:
    bool init(void);
    void update();

protected:
    virtual bool hw_init() = 0;
    virtual bool hw_update() = 0;
    virtual bool set_pixel(uint16_t x, uint16_t y) = 0;
    virtual bool clear_pixel(uint16_t x, uint16_t y) = 0;

private:
    void draw_char(uint16_t x, uint16_t y, const char c);
    void draw_text(uint16_t x, uint16_t y, const char *c);
    void update_all();
    void update_arm();
    void update_prearm();
    void update_gps();
    void update_gps_sats();
    void update_ekf();

    bool _healthy;
    bool _need_update;

    struct display_state {
        uint32_t gps_status         : 3;    // 0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
        uint32_t gps_num_sats       : 6;    // number of sats
        uint32_t armed              : 1;    // 0 = disarmed, 1 = armed
        uint32_t pre_arm_check      : 1;    // 0 = failing checks, 1 = passed
        uint32_t ekf_bad            : 1;    // 1 if ekf is reporting problems
    } _flags;
};

