#pragma once

#include "NotifyDevice.h"

#define ROW(Y)    ((Y * 10) + 6)
#define COLUMN(X) ((X *  7) + 0)

#define DISPLAY_MESSAGE_SIZE 18

class Display_Backend;

class Display: public NotifyDevice {
public:
    friend class Display_Backend;

    bool init(void);
    void update();

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

    Display_Backend *_driver;

    bool _healthy;

    uint8_t _mstartpos;
    uint8_t _movedelay;
    uint8_t _screenpage;
};

