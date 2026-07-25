/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the STMicroelectronics STM3210E-EVAL evaluation board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM3210E_EVAL
#define BOARD_NAME              "ST STM3210E-EVAL"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            8000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 * Note: Older board revisions should define STM32F10X_HD instead, please
 *       verify the STM32 model mounted on your board. The change also
 *       affects your linker script.
 */
#define STM32F103xG

/*
 * IO pins assignments.
 */
#define GPIOA_WAKEUP_BUTTON     0

#define GPIOB_SC_3V_5V          0
#define GPIOB_SPI1_CS           2
#define GPIOB_TEMP_INT          5
#define GPIOB_USB_DISC          14

#define GPIOC_SC_CMDVCC         6
#define GPIOC_SC_OFF            7
#define GPIOC_TAMPER_BUTTON     13

#define GPIOD_JOY_DOWN          3

#define GPIOF_LED1              6
#define GPIOF_LED2              7
#define GPIOF_LED3              8
#define GPIOF_LED4              9
#define GPIOF_SD_DETECT         11

#define GPIOG_JOY_SEL           7
#define GPIOG_USER_BUTTON       8
#define GPIOG_JOY_RIGHT         13
#define GPIOG_JOY_LEFT          14
#define GPIOG_JOY_UP            15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_ANALOG(n)           (0 << (((n) & 7) * 4))
#define PIN_OUTPUT_PP_10(n)     (1 << (((n) & 7) * 4))
#define PIN_OUTPUT_PP_2(n)      (2 << (((n) & 7) * 4))
#define PIN_OUTPUT_PP_50(n)     (3 << (((n) & 7) * 4))
#define PIN_INPUT(n)            (4 << (((n) & 7) * 4))
#define PIN_OUTPUT_OD_10(n)     (5 << (((n) & 7) * 4))
#define PIN_OUTPUT_OD_2(n)      (6 << (((n) & 7) * 4))
#define PIN_OUTPUT_OD_50(n)     (7 << (((n) & 7) * 4))
#define PIN_INPUT_PUD(n)        (8 << (((n) & 7) * 4))
#define PIN_ALTERNATE_PP_10(n)  (9 << (((n) & 7) * 4))
#define PIN_ALTERNATE_PP_2(n)   (10 << (((n) & 7) * 4))
#define PIN_ALTERNATE_PP_50(n)  (11 << (((n) & 7) * 4))
#define PIN_ALTERNATE_OD_10(n)  (13 << (((n) & 7) * 4))
#define PIN_ALTERNATE_OD_2(n)   (14 << (((n) & 7) * 4))
#define PIN_ALTERNATE_OD_50(n)  (15 << (((n) & 7) * 4))
#define PIN_UNDEFINED(n)        PIN_INPUT_PUD(n)

/*
 * Port A setup.
 */
#define VAL_GPIOACRL    (PIN_INPUT(0)           | /* Wakeup Button.     */  \
                         PIN_OUTPUT_PP_50(1)    | /* USART2_RTS.        */  \
                         PIN_ALTERNATE_PP_50(2) | /* USART2_TX.         */  \
                         PIN_INPUT(3)           | /* USART2_RX.         */  \
                         PIN_UNDEFINED(4)       |                           \
                         PIN_ALTERNATE_PP_50(5) | /* SPI1_SCK.          */  \
                         PIN_INPUT(6)           | /* SPI1_MISO.         */  \
                         PIN_ALTERNATE_PP_50(7))  /* SPI1_MOSI.         */
#define VAL_GPIOACRH    (PIN_ALTERNATE_PP_50(8) | /* MCO.               */  \
                         PIN_ALTERNATE_PP_50(9) | /* USART1_TX.         */  \
                         PIN_INPUT(10)          | /* USART1_RX.         */  \
                         PIN_INPUT_PUD(11)      | /* USB_DM.            */  \
                         PIN_INPUT_PUD(12)      | /* USB_DP.            */  \
                         PIN_INPUT(13)          | /* TMS.               */  \
                         PIN_INPUT(14)          | /* TCK.               */  \
                         PIN_INPUT(15))           /* TDI.               */
#define VAL_GPIOAODR    0xFFFFFFFF

/*
 * Port B setup.
 */
#define VAL_GPIOBCRL    (PIN_OUTPUT_PP_50(0)    | /* SmartCard_3/5V.    */  \
                         PIN_INPUT_PUD(1)       | /* Unconnected.       */  \
                         PIN_OUTPUT_PP_50(2)    | /* SPI1_CS.           */  \
                         PIN_INPUT(3)           | /* TDO.               */  \
                         PIN_INPUT(4)           | /* TRST.              */  \
                         PIN_INPUT_PUD(5)       | /* Temp.Sensor INT.   */  \
                         PIN_ALTERNATE_OD_50(6) | /* I2C1_SCK.          */  \
                         PIN_ALTERNATE_OD_50(7))  /* I2C1_SDA.          */
#define VAL_GPIOBCRH    (PIN_INPUT(8)           | /* CAN_RX.            */  \
                         PIN_ALTERNATE_PP_50(9) | /* CAN_TX.            */  \
                         PIN_ALTERNATE_OD_50(10)| /* SmartCard IO.      */  \
                         PIN_OUTPUT_PP_50(11)   | /* SmartCard RST.     */  \
                         PIN_ALTERNATE_PP_50(12)| /* SmartCard CLK.     */  \
                         PIN_UNDEFINED(13)      |                           \
                         PIN_OUTPUT_PP_50(14)   | /* USB disconnect.    */  \
                         PIN_UNDEFINED(15))
#define VAL_GPIOBODR    0xFFFFFFFF

/*
 * Port C setup.
 */
#define VAL_GPIOCCRL    (PIN_UNDEFINED(0)       |                           \
                         PIN_UNDEFINED(1)       |                           \
                         PIN_UNDEFINED(2)       |                           \
                         PIN_UNDEFINED(3)       |                           \
                         PIN_ANALOG(4)          | /* Potentiometer.     */  \
                         PIN_UNDEFINED(5)       |                           \
                         PIN_OUTPUT_PP_50(6)    | /* SmartCard CMDVCC.  */  \
                         PIN_INPUT(7))            /* SmartCard OFF.     */
#define VAL_GPIOCCRH    (PIN_ALTERNATE_PP_50(8) | /* SDIO D0.           */  \
                         PIN_ALTERNATE_PP_50(9) | /* SDIO D1.           */  \
                         PIN_ALTERNATE_PP_50(10)| /* SDIO D2.           */  \
                         PIN_ALTERNATE_PP_50(11)| /* SDIO D3.           */  \
                         PIN_ALTERNATE_PP_50(12)| /* SDIO CLK.          */  \
                         PIN_INPUT(13)          | /* Tamper Button.     */  \
                         PIN_INPUT(14)          | /* OSC IN.            */  \
                         PIN_INPUT(15))           /* OSC OUT.           */
#define VAL_GPIOCODR    0xFFFFFFFF

/*
 * Port D setup
 */
#define VAL_GPIODCRL    (PIN_ALTERNATE_PP_50(0) | /* FSMC_D2.           */  \
                         PIN_ALTERNATE_PP_50(1) | /* FSMC_D3.           */  \
                         PIN_ALTERNATE_PP_50(2) | /* SDIO CMD.          */  \
                         PIN_INPUT(3)           | /* Joy Down.          */  \
                         PIN_ALTERNATE_PP_50(4) | /* FSMC_NOE.          */  \
                         PIN_ALTERNATE_PP_50(5) | /* FSMC_NWE.          */  \
                         PIN_INPUT(6)           | /* FSMC_NWAIT.        */  \
                         PIN_ALTERNATE_PP_50(7))  /* FSMC_NCE2.         */
#define VAL_GPIODCRH    (PIN_ALTERNATE_PP_50(8) | /* FSMC_D13.          */  \
                         PIN_ALTERNATE_PP_50(9) | /* FSMC_D14.          */  \
                         PIN_ALTERNATE_PP_50(10)| /* FSMC_D15.          */  \
                         PIN_ALTERNATE_PP_50(11)| /* FSMC_A16.          */  \
                         PIN_ALTERNATE_PP_50(12)| /* FSMC_A17.          */  \
                         PIN_ALTERNATE_PP_50(13)| /* FSMC_A18.          */  \
                         PIN_ALTERNATE_PP_50(14)| /* FSMC_D0.           */  \
                         PIN_ALTERNATE_PP_50(15)) /* FSMC_D1.           */
#define VAL_GPIODODR    0xFFFFFFFF

/*
 * Port E setup.
 */
#define VAL_GPIOECRL    (PIN_ALTERNATE_PP_50(0) | /* FSMC_NBL0.         */  \
                         PIN_ALTERNATE_PP_50(1) | /* FSMC_NBL1.         */  \
                         PIN_ALTERNATE_PP_50(2) | /* FSMC_A23.          */  \
                         PIN_ALTERNATE_PP_50(3) | /* FSMC_A19.          */  \
                         PIN_ALTERNATE_PP_50(4) | /* FSMC_A20.          */  \
                         PIN_ALTERNATE_PP_50(5) | /* FSMC_A21.          */  \
                         PIN_ALTERNATE_PP_50(6) | /* FSMC_A22.          */  \
                         PIN_ALTERNATE_PP_50(7))  /* FSMC_D4.           */
#define VAL_GPIOECRH    (PIN_ALTERNATE_PP_50(8) | /* FSMC_D5.           */  \
                         PIN_ALTERNATE_PP_50(9) | /* FSMC_D6.           */  \
                         PIN_ALTERNATE_PP_50(10)| /* FSMC_D7.           */  \
                         PIN_ALTERNATE_PP_50(11)| /* FSMC_D8.           */  \
                         PIN_ALTERNATE_PP_50(12)| /* FSMC_D9.           */  \
                         PIN_ALTERNATE_PP_50(13)| /* FSMC_D10.          */  \
                         PIN_ALTERNATE_PP_50(14)| /* FSMC_D11.          */  \
                         PIN_ALTERNATE_PP_50(15)) /* FSMC_D12.          */
#define VAL_GPIOEODR    0xFFFFFFFF

/*
 * Port F setup.
 */
#define VAL_GPIOFCRL    (PIN_ALTERNATE_PP_50(0) | /* FSMC_A0.           */  \
                         PIN_ALTERNATE_PP_50(1) | /* FSMC_A1.           */  \
                         PIN_ALTERNATE_PP_50(2) | /* FSMC_A2.           */  \
                         PIN_ALTERNATE_PP_50(3) | /* FSMC_A3.           */  \
                         PIN_ALTERNATE_PP_50(4) | /* FSMC_A4.           */  \
                         PIN_ALTERNATE_PP_50(5) | /* FSMC_A5.           */  \
                         PIN_OUTPUT_PP_50(6)    | /* LED1.              */  \
                         PIN_OUTPUT_PP_50(7))     /* LED2.              */
#define VAL_GPIOFCRH    (PIN_OUTPUT_PP_50(8)    | /* LED3.              */  \
                         PIN_OUTPUT_PP_50(9)    | /* LED4.              */  \
                         PIN_UNDEFINED(10)      |                           \
                         PIN_INPUT_PUD(11)      | /* SDCard detect.     */  \
                         PIN_ALTERNATE_PP_50(12)| /* FSMC_A6.           */  \
                         PIN_ALTERNATE_PP_50(13)| /* FSMC_A7.           */  \
                         PIN_ALTERNATE_PP_50(14)| /* FSMC_A8.           */  \
                         PIN_ALTERNATE_PP_50(15)) /* FSMC_A9.           */
#define VAL_GPIOFODR    0xFFFFFC3F

/*
 * Port G setup.
 */
#define VAL_GPIOGCRL    (PIN_ALTERNATE_PP_50(0) | /* FSMC_A10.          */  \
                         PIN_ALTERNATE_PP_50(1) | /* FSMC_A11.          */  \
                         PIN_ALTERNATE_PP_50(2) | /* FSMC_A12.          */  \
                         PIN_ALTERNATE_PP_50(3) | /* FSMC_A13.          */  \
                         PIN_ALTERNATE_PP_50(4) | /* FSMC_A14.          */  \
                         PIN_ALTERNATE_PP_50(5) | /* FSMC_A15.          */  \
                         PIN_INPUT(6)           | /* FSMC_INT2.         */  \
                         PIN_INPUT(7))            /* Joy Select.        */
#define VAL_GPIOGCRH    (PIN_INPUT(8)           | /* User Button.       */  \
                         PIN_ALTERNATE_PP_50(9) | /* FSMC_NE2.          */  \
                         PIN_ALTERNATE_PP_50(10)| /* FSMC_NE3.          */  \
                         PIN_OUTPUT_PP_50(11)   | /* Audio PDN.         */  \
                         PIN_ALTERNATE_PP_50(12)| /* FSMC_NE4.          */  \
                         PIN_INPUT(13)          | /* Joy Right.         */  \
                         PIN_INPUT(14)          | /* Joy Left.          */  \
                         PIN_INPUT(15))           /* Joy Up.            */
#define VAL_GPIOGODR    0xFFFFF7FF

/*
 * USB bus activation macro, required by the USB driver.
 */
#define usb_lld_connect_bus(usbp) palClearPad(GPIOB, GPIOB_USB_DISC)

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp) palSetPad(GPIOB, GPIOB_USB_DISC)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
