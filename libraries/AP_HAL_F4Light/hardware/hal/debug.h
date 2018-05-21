#pragma once

//DBG_CR
#define DBGMCU_SLEEP                 (1L<<0)
#define DBGMCU_STOP                  (1L<<1)
#define DBGMCU_STANDBY               (1L<<2)

//DBG_APB1_FZ
#define DBGMCU_TIM2_STOP             (1L<<0)
#define DBGMCU_TIM3_STOP             (1L<<1)
#define DBGMCU_TIM4_STOP             (1L<<2)
#define DBGMCU_TIM5_STOP             (1L<<3)
#define DBGMCU_TIM6_STOP             (1L<<4)
#define DBGMCU_TIM7_STOP             (1L<<5)
#define DBGMCU_TIM12_STOP            (1L<<6)
#define DBGMCU_TIM13_STOP            (1L<<7)
#define DBGMCU_TIM14_STOP            (1L<<8)
#define DBGMCU_RTC_STOP              (1L<<9)
// 10 reserved
#define DBGMCU_WWDG_STOP             (1L<<11)
#define DBGMCU_IWDG_STOP             (1L<<12)
//13..20 reserved
#define DBGMCU_I2C1_SMBUS_TIMEOUT    (1L<<21)
#define DBGMCU_I2C2_SMBUS_TIMEOUT    (1L<<22)
#define DBGMCU_I2C3_SMBUS_TIMEOUT    (1L<<23)
//24 reserved
#define DBGMCU_CAN1_STOP             (1L<<25)
#define DBGMCU_CAN2_STOP             (1L<<26)

//DBG_APB2_FZ
#define DBGMCU_TIM1_STOP             (1L<<0)
#define DBGMCU_TIM8_STOP             (1L<<1)
#define DBGMCU_TIM9_STOP             (1L<<16)
#define DBGMCU_TIM10_STOP            (1L<<17)
#define DBGMCU_TIM11_STOP            (1L<<18)
