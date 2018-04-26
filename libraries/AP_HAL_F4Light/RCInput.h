#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_F4Light.h"
#include "UARTDriver.h"
#include <usart.h>
#include "RC_parser.h"
#include "Config.h"
#include "c++.h"


#define BOARD_RC_FAILSAFE 2 // flag that failsafe is enabled

#define  RC_DEAD_TIME 60000 // 60 seconds no data changes

#ifndef BOARD_SPEKTRUM_RX_PIN
 #ifdef BOARD_DSM_USART
    #define BOARD_SPEKTRUM_RX_PIN (BOARD_DSM_USART->rx_pin)
 #endif
#endif

enum BOARD_RC_MODE {
    BOARD_RC_NONE    = 0,
    BOARD_RC_SBUS    = 1,
    BOARD_RC_DSM     = 2,
    BOARD_RC_SUMD    = 4,
    BOARD_RC_SBUS_NI = 8,
};

#define MAX_RC_PARSERS 6


class F4Light::RCInput : public AP_HAL::RCInput {
public:
    RCInput();
    void init()  override;
    static void late_init(uint8_t b);

    uint16_t read(uint8_t ch) override;
    uint8_t  read(uint16_t* periods, uint8_t len) override;
    
    bool     new_input() override;
    uint8_t  num_channels() override;

    bool rc_bind(int dsmMode) override;

    static uint16_t max_num_pulses; // for statistics
        
private:
    static _parser * parsers[MAX_RC_PARSERS];
    static uint8_t num_parsers;
    static uint8_t _last_read_from;
    
    static bool is_PPM;

    static uint64_t _last_read;
    static uint8_t  _valid_channels;


    uint16_t _read_dsm(uint8_t ch);
    uint16_t _read_ppm(uint8_t ch,uint8_t n);
    
    static uint16_t last_4;
    
    static bool rc_failsafe_enabled;    

    static bool fs_flag, aibao_fs_flag;
};


