#ifndef _BOARD_STM32V1F4_H_
#define _BOARD_STM32V1F4_H_

#include "hal_types.h"
#include "wirish_types.h"

void boardInit(void);

//#define Port2Pin(port, bit) ((port-'A')*16+bit)

/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
 */
#define __CM4_REV                 0x0001  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present      */

#define CYCLES_PER_MICROSECOND  168
#define SYSTICK_RELOAD_VAL      (CYCLES_PER_MICROSECOND*1000-1)

#undef  STM32_PCLK1
#undef  STM32_PCLK2
#define STM32_PCLK1   (CYCLES_PER_MICROSECOND*1000000/4)
#define STM32_PCLK2   (CYCLES_PER_MICROSECOND*1000000/2)

#define BOARD_LED_PIN           36
#define BOARD_BUTTON_PIN        19 // PC13 - not used pin!

#define BOARD_NR_USARTS         5
#define BOARD_USART1_TX_PIN     23 
#define BOARD_USART1_RX_PIN     24 
#define BOARD_USART2_TX_PIN     200
#define BOARD_USART2_RX_PIN     200
#define BOARD_USART3_TX_PIN     0
#define BOARD_USART3_RX_PIN     100
#define BOARD_UART4_TX_PIN      200
#define BOARD_UART4_RX_PIN      200
#define BOARD_UART5_TX_PIN      200
#define BOARD_UART5_RX_PIN      200
       
#define BOARD_NR_SPI            3
#define BOARD_SPI1_NSS_PIN      D51
#define BOARD_SPI1_SCK_PIN      D52
#define BOARD_SPI1_MISO_PIN     D53
#define BOARD_SPI1_MOSI_PIN     D54
#define BOARD_SPI1_CS_BR_PIN    51
#define BOARD_SPI2_NSS_PIN      200
#define BOARD_SPI2_SCK_PIN      200
#define BOARD_SPI2_MISO_PIN     200
#define BOARD_SPI2_MOSI_PIN     200
#define BOARD_SPI3_NSS_PIN      104
#define BOARD_SPI3_MOSI_PIN     18
#define BOARD_SPI3_MISO_PIN     17
#define BOARD_SPI3_CS_DF_PIN    104
#define BOARD_SPI3_SCK_PIN      16

#define BOARD_RFM22B_CS_PIN     103 // PA15 CS_RFM22B
#define BOARD_RFM22B_INT_PIN    26  // PD2

#define BOARD_HMC5883_DRDY_PIN  38  // PB7

#define MPU6000_CS_PIN		51

#define BOARD_NR_GPIO_PINS      106
#define BOARD_NR_PWM_PINS       18
#define BOARD_NR_ADC_PINS       21
#define BOARD_NR_USED_PINS      43
#define BOARD_JTMS_SWDIO_PIN    101
#define BOARD_JTCK_SWCLK_PIN    102
#define BOARD_JTDI_PIN          103
#define BOARD_JTDO_PIN          104
#define BOARD_NJTRST_PIN        105

#define BOARD_USB_DISC_DEV      GPIOC
#define BOARD_USB_DISC_BIT      7

#define ADC_CHIP_SELECT D77

/* ADC *****************************************************/
/*
#define BOARD_ADC_CH0 D47
#define BOARD_ADC_CH1 D51
#define BOARD_ADC_CH2 D6
#define BOARD_ADC_CH3 D7
#define BOARD_ADC_CH4 D8
#define BOARD_ADC_CH5 D9
#define BOARD_ADC_CH6 D10
#define BOARD_ADC_CH7 D11
*/
#ifndef MOTOR_PWM_FREQ
  #define MOTOR_PWM_FREQ 60
#endif


#endif
