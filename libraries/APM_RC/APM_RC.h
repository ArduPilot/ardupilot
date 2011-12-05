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

#define MSK_CH_1 (1 << CH_1)
#define MSK_CH_2 (1 << CH_2)
#define MSK_CH_3 (1 << CH_3)
#define MSK_CH_4 (1 << CH_4)
#define MSK_CH_5 (1 << CH_5)
#define MSK_CH_6 (1 << CH_6)
#define MSK_CH_7 (1 << CH_7)
#define MSK_CH_8 (1 << CH_8)
#define MSK_CH_9 (1 << CH_9)
#define MSK_CH_10 (1 << CH_10)
#define MSK_CH_11 (1 << CH_11)



#define NUM_CHANNELS 8


class Arduino_Mega_ISR_Registry;

class APM_RC_Class
{
  public:
	virtual void Init( Arduino_Mega_ISR_Registry * isr_reg ) = 0;
	virtual void OutputCh(uint8_t ch, uint16_t pwm) = 0;
	virtual uint16_t InputCh(uint8_t ch) = 0;
	virtual uint8_t GetState() = 0;
	virtual void clearOverride(void) = 0;
    virtual void Force_Out() = 0;
    virtual void SetFastOutputChannels( uint32_t channelmask ) = 0;
};

#include "APM_RC_APM1.h"
#include "APM_RC_APM2.h"

#endif
