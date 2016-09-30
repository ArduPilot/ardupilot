#pragma once

#include "NotifyDevice.h"

#define ROW(Y)    ((Y * 10) + 6)
#define COLUMN(X) ((X *  7) + 0)

#define DISPLAY_MESSAGE_SIZE 18

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

    uint8_t _mstartpos;
    uint8_t _movedelay;
};

