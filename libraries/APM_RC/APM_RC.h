#ifndef __APM_RC_H__
#define __APM_RC_H__


#include <inttypes.h>


// Radio channels

// Note channels are from 0!

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10


#define NUM_CHANNELS 8



class Arduino_Mega_ISR_Registry;

class APM_RC_Class

{

public:

    virtual void            Init( Arduino_Mega_ISR_Registry * isr_reg ) = 0;
    virtual void            OutputCh(uint8_t ch, uint16_t pwm) = 0;
    virtual uint16_t        OutputCh_current(uint8_t ch) = 0;
    virtual uint16_t        InputCh(uint8_t ch) = 0;
    virtual uint8_t         GetState() = 0;
    virtual void            clearOverride(void) = 0;
    virtual void            Force_Out() = 0;
    virtual void            SetFastOutputChannels( uint32_t channelmask, uint16_t speed_hz = 400 ) = 0;
    virtual void            enable_out(uint8_t) = 0;
    virtual void            disable_out(uint8_t) = 0;

    virtual void            Force_Out0_Out1(void) = 0;
    virtual void            Force_Out2_Out3(void) = 0;
    virtual void            Force_Out6_Out7(void) = 0;

protected:
    uint16_t                _map_speed(uint16_t speed_hz) {
        return 2000000UL / speed_hz;
    }

};



#include "APM_RC_APM1.h"
#include "APM_RC_APM2.h"


#endif

