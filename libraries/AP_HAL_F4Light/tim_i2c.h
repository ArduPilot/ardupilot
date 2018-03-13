#pragma once

#include <stdbool.h>
#include <stm32f4xx.h>
#include <hal.h>
#include "systick.h"
#include "Scheduler.h"
#include "timer.h"

//#define SI2C_DEBUG 1
//#define SI2C_PROF 1



class Soft_I2C {
public:

    typedef enum STATE {
        START,
        // address 
        A_0L,
        A_0H,
        A_1L,
        A_1H,
        A_2L,
        A_2H,
        A_3L,
        A_3H,
        A_4L,
        A_4H,
        A_5L,
        A_5H,
        A_6L,
        A_6H,
        A_7L,
        A_7H,
        A_AL,
        A_AH,
        // byte write
        W_0L,
        W_0H,
        W_1L,
        W_1H,
        W_2L,
        W_2H,
        W_3L,
        W_3H,
        W_4L,
        W_4H,
        W_5L,
        W_5H,
        W_6L,
        W_6H,
        W_7L,
        W_7H,
        W_AL,
        W_AH,
        
        STOP,
        STOP2,
        RESTART,
        RESTART2, // 2nd stage
        DUMMY,
        // byte read after restart
        R_0L,
        R_0H,
        R_1L,
        R_1H,
        R_2L,
        R_2H,
        R_3L,
        R_3H,
        R_4L,
        R_4H,
        R_5L,
        R_5H,
        R_6L,
        R_6H,
        R_7L,
        R_7H,
        R_AL,
        R_AH,
    } State;

#ifdef SI2C_DEBUG
    typedef struct SI2C_STATE {
        uint32_t time;
        State state;
        unsigned int sda;
        unsigned int f_sda;
    } SI2C_State;
#endif 



    Soft_I2C();

    void init();
    void init_hw( const gpio_dev *scl_dev, uint8_t scl_bit, const gpio_dev *sda_dev, uint8_t sda_bit, const timer_dev *tim);
    
    uint8_t writeBuffer( uint8_t addr_, uint8_t len_, const uint8_t *data);
    uint8_t read( uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
    uint8_t transfer(uint8_t  addr, uint8_t  send_len, const uint8_t *send, uint8_t len, uint8_t *buf);

    bool bus_reset(void);
    
    void set_low_speed(bool s) {  }


private:
    void tick(); // ISR

    bool    _start(void);
    uint8_t wait_done();

    const gpio_dev *_scl_dev;
    uint8_t         _scl_bit;
    const gpio_dev *_sda_dev;
    uint8_t         _sda_bit;
    const timer_dev * _timer;

    volatile GPIO_TypeDef *scl_port; // for quick direct access to hardware
    uint16_t               scl_pin;
    volatile GPIO_TypeDef *sda_port;
    uint16_t               sda_pin;

    volatile bool done;
    volatile uint8_t result;
    volatile State state;
    bool f_sda;    // what line we touch
    bool wait_scl; // flag - we wait for SCL stretching
    bool was_restart;

    uint8_t data;  // data byte to output / from input

    const uint8_t *send;
    uint8_t        send_len;
    uint8_t       *recv;
    uint8_t        recv_len;
    uint8_t        _addr;

    uint16_t bit_time;

#ifdef SI2C_DEBUG
    #define SI2C_LOG_SIZE 199
    static SI2C_State log[SI2C_LOG_SIZE];
    static uint16_t log_ptr;
#endif

#ifdef SI2C_PROF
    static uint64_t full_time;
    static uint32_t int_count;
#endif

};

