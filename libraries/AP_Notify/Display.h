#pragma once

#include "NotifyDevice.h"

#define ROW(Y) ((Y * 11) + 2)
#define COLUMN(X) ((X * 7) + 1)

#define DISPLAY_MESSAGE_SIZE 18
#define DISPLAY_TEXT_SIZE 51

class Display: public NotifyDevice {
public:
    bool init(void);
    void update();

protected:
    virtual bool hw_init() = 0;
    virtual bool hw_update() = 0;
    virtual bool set_pixel(uint16_t x, uint16_t y) = 0;
    virtual bool clear_pixel(uint16_t x, uint16_t y) = 0;
    virtual bool clear_screen() = 0;

private:
    void draw_char(uint16_t x, uint16_t y, const char c);
    void draw_text(uint16_t x, uint16_t y, const char *c);
    void update_all();
    void update_arm(uint8_t r);
    void update_prearm(uint8_t r);
    void update_gps(uint8_t r);
    void update_gps_sats(uint8_t r);
    void update_ekf(uint8_t r);
    void update_battery(uint8_t r);
    void update_mode(uint8_t r);
    void update_text(uint8_t r);

    bool _healthy;
    bool _need_update;

    uint8_t mstartpos;
    uint8_t movedelay;

    struct display_state {
        uint32_t gps_status         : 3;    // 0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
        uint32_t gps_num_sats       : 6;    // number of sats
        uint32_t armed              : 1;    // 0 = disarmed, 1 = armed
        uint32_t pre_arm_check      : 1;    // 0 = failing checks, 1 = passed
        uint32_t ekf_bad            : 1;    // 1 if ekf is reporting problems
        uint32_t control_mode       : 6;    // control_mode
    } _flags;
};

