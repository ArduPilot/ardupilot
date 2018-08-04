#pragma once

#include "AP_Baro_Backend.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MAMBO

class AP_Baro_InputEvent : public AP_Baro_Backend {
public:
    AP_Baro_InputEvent(AP_Baro &);
    static AP_Baro_Backend *probe(AP_Baro &baro);
    void update() override;
    void tick();
private:
    bool _init();
    uint8_t _instance;
    float _pressure;
    int _fd;
    bool _data_avail;
};

#endif
