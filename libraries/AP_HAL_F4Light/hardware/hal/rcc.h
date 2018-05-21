#ifndef _RCC_H
#define _RCC_H

#include "hal_types.h"
#include "bitband.h"

#ifdef __cplusplus
  extern "C" {
#endif


#define RCC_RTCCLKSource_LSI             (1L<<9)

#define RCC_HSE_OFF                      (0)
#define RCC_HSE_ON                       (1)

#define RCC_BIT_HSE_RDY                  (RCC_CR_HSERDY)

#define RCC_APB1_bit_TIM2              (1L<<0)
#define RCC_APB1_bit_TIM3              (1L<<1)
#define RCC_APB1_bit_TIM4              (1L<<2)
#define RCC_APB1_bit_TIM5              (1L<<3)
#define RCC_APB1_bit_TIM6              (1L<<4)
#define RCC_APB1_bit_TIM7              (1L<<5)
#define RCC_APB1_bit_TIM12             (1L<<6)
#define RCC_APB1_bit_TIM13             (1L<<7)
#define RCC_APB1_bit_TIM14             (1L<<8)
// 9..10 reserved
#define RCC_APB1_bit_WWDG              (1L<<11)
// 12..13 reserved
#define RCC_APB1_bit_SPI2              (1L<<14)
#define RCC_APB1_bit_SPI3              (1L<<15)
// 16 reserved
#define RCC_APB1_bit_USART2            (1L<<17)
#define RCC_APB1_bit_USART3            (1L<<18)
#define RCC_APB1_bit_UART4             (1L<<19)
#define RCC_APB1_bit_UART5             (1L<<20)
#define RCC_APB1_bit_I2C1              (1L<<21)
#define RCC_APB1_bit_I2C2              (1L<<22)
#define RCC_APB1_bit_I2C3              (1L<<23)
// 24 reserved
#define RCC_APB1_bit_CAN1              (1L<<25)
#define RCC_APB1_bit_CAN2              (1L<<26)
// 27 reserved
#define RCC_APB1_bit_PWR               (1L<<28)
#define RCC_APB1_bit_DAC               (1L<<29)
#define RCC_APB1_bit_UART7             (1L<<30)
#define RCC_APB1_bit_UART8             (1L<<31)

#define RCC_APB2_bit_TIM1              (1L<<0)
#define RCC_APB2_bit_TIM8              (1L<<1)
//2..3 reserved
#define RCC_APB2_bit_USART1            (1L<<4)
#define RCC_APB2_bit_USART6            (1L<<5)
//6..7 reserved
#define RCC_APB2_bit_ADC1              (1L<<8)
#define RCC_APB2_bit_ADC2              (1L<<9)
#define RCC_APB2_bit_ADC3              (1L<<10)
#define RCC_APB2_bit_ADC               (RCC_APB2_bit_ADC1 | RCC_APB2_bit_ADC2 | RCC_APB2_bit_ADC3)
#define RCC_APB2_bit_SDIO              (1L<<11)
#define RCC_APB2_bit_SPI1              (1L<<12)
#define RCC_APB2_bit_SPI4              (1L<<13)
#define RCC_APB2_bit_SYSCFG            (1L<<14)
// 15 reserved
#define RCC_APB2_bit_TIM9              (1L<<16)
#define RCC_APB2_bit_TIM10             (1L<<17)
#define RCC_APB2_bit_TIM11             (1L<<18)
// 19 reserved
#define RCC_APB2_bit_SPI5              (1L<<20)
#define RCC_APB2_bit_SPI6              (1L<<21)

#define RCC_AHB1_bit_GPIOA             (1L<<0)
#define RCC_AHB1_bit_GPIOB             (1L<<1)
#define RCC_AHB1_bit_GPIOC             (1L<<2)
#define RCC_AHB1_bit_GPIOD             (1L<<3)
#define RCC_AHB1_bit_GPIOE             (1L<<4)
#define RCC_AHB1_bit_GPIOF             (1L<<5)
#define RCC_AHB1_bit_GPIOG             (1L<<6)
#define RCC_AHB1_bit_GPIOH             (1L<<7)
#define RCC_AHB1_bit_GPIOI             (1L<<8)
// 9..11 reserved
#define RCC_AHB1_bit_CRC               (1L<<12)
//13..14 reserved
#define RCC_AHB1_bit_FLITF             (1L<<15)
#define RCC_AHB1_bit_SRAM1             (1L<<16)
#define RCC_AHB1_bit_SRAM2             (1L<<17)
#define RCC_AHB1_bit_BKPSRAM           (1L<<18)
#define RCC_AHB1_bit_SRAM3             (1L<<19)
#define RCC_AHB1_bit_CCMDATARAMEN      (1L<<20)
#define RCC_AHB1_bit_DMA1              (1L<<21)
#define RCC_AHB1_bit_DMA2              (1L<<22)
//23..24 reserved
#define RCC_AHB1_bit_ETH_MAC           (1L<<25)
#define RCC_AHB1_bit_ETH_MAC_Tx        (1L<<26)
#define RCC_AHB1_bit_ETH_MAC_Rx        (1L<<27)
#define RCC_AHB1_bit_ETH_MAC_PTP       (1L<<28)
#define RCC_AHB1_bit_OTG_HS            (1L<<29)
#define RCC_AHB1_bit_OTG_HS_ULPI       (1L<<30)

#define RCC_AHB2_bit_DCMI              (1L<<0)
#define RCC_AHB2_bit_CRYP              (1L<<4)
#define RCC_AHB2_bit_HASH              (1L<<5)
#define RCC_AHB2_bit_RNG               (1L<<6)
#define RCC_AHB2_bit_OTG_FS            (1L<<7)


typedef struct {
  uint32_t HCLK_Frequency;   //  HCLK  clock frequency, Hz  
  uint32_t PCLK1_Frequency;  //  PCLK1 clock frequency, Hz 
  uint32_t PCLK2_Frequency;  //  PCLK2 clock frequency, Hz 
} RCC_Clocks_t;


void RCC_configRTC(uint32_t RCC_RTCCLKSource);
bool RCC_WaitForHSEStartUp(void);
bool RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_enableHSE(uint8_t hse);

void RCC_GetClocksFreq(RCC_Clocks_t* RCC_Clocks);

void RCC_doAPB1_reset(uint32_t dev_bit);
void RCC_doAPB2_reset(uint32_t dev_bit);
void RCC_doAHB1_reset(uint32_t dev_bit);


static inline void RCC_enableRTCclk(bool enable){  *bb_perip(&(RCC->BDCR), 15) = enable?1:0; }

static inline void RCC_enableAHB1_clk(uint32_t dev_bit) {    RCC->AHB1ENR |= dev_bit; }
static inline void RCC_enableAHB2_clk(uint32_t dev_bit) {    RCC->AHB2ENR |= dev_bit; }
static inline void RCC_enableAPB1_clk(uint32_t dev_bit) {    RCC->APB1ENR |= dev_bit; }
static inline void RCC_enableAPB2_clk(uint32_t dev_bit) {    RCC->APB2ENR |= dev_bit; }

static inline void RCC_disableAHB2_clk(uint32_t dev_bit){    RCC->AHB2ENR &= ~dev_bit;}


#ifdef __cplusplus
  }
#endif


#endif
