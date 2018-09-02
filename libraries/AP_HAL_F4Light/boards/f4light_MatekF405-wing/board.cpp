#ifndef BOARD_STM32V1F4
#define BOARD_STM32V1F4

#include <boards.h>
#include "../../SPIDevice.h"
#include <AP_Common/AP_Common.h>

using namespace F4Light;

/*
    LQFP64
    
    PA0-15
    PB0-15
    PC0-12
    PD2
    

*/

extern const stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS] __FLASH__ = {

    /* Top header */
/*
    const gpio_dev  * const gpio_device;      < Maple pin's GPIO device 
    const timer_dev * const timer_device;     < Pin's timer device, if any. 
    const adc_dev   * const adc_device;       < ADC device, if any. 
    uint8_t gpio_bit;             < Pin's GPIO port bit. 
    uint8_t timer_channel;        < Timer channel, or 0 if none. 
    uint8_t adc_channel;          < Pin ADC channel, or nADC if none. 
*/

    {&gpiob,   NULL, NULL, 10, NO_CH,     nADC}, /* D0/PB10  0 I2C2_SCL */
    {&gpiob,   NULL, NULL,  2, NO_CH,     nADC}, /* D1/PB2   1 (BOOT1) */
    {&gpiob,   NULL, NULL, 12, NO_CH,     nADC}, /* D2/PB12  2 OSD_CS */
    {&gpiob,   NULL, NULL, 13, NO_CH,     nADC}, /* D3/PB13  3 SPI2_SCK */
    {&gpiob,&timer12,NULL, 14, TIMER_CH1, nADC}, /* D4/PB14  4 SERVO 7 */
    {&gpiob,&timer12,NULL, 15, TIMER_CH2, nADC}, /* D5/PB15  5 SERVO 8 */
    {&gpioc,   NULL,&_adc1, 0, NO_CH,       10}, /* D6/PC0   6 Volt sensor */
    {&gpioc,   NULL,&_adc1, 1, NO_CH,       11}, /* D7/PC1   7 curr sensor */
    {&gpioc,   NULL,&_adc1, 2, NO_CH,       12}, /* D8/PC2   8 SPI2_MISO */
    {&gpioc,   NULL,&_adc1, 3, NO_CH,       13}, /* D9/PC3   9 SPI2_MOSI */
    {&gpioc,   NULL,&_adc1, 4, NO_CH,       14}, /* D10/PC4  10 MPU6000_DRDY */
    {&gpioc,   NULL,&_adc1, 5, NO_CH,       15}, /* D11/PC5  1  RSSI sens */
    {&gpioc, &timer8,NULL,  6, TIMER_CH1, nADC}, /* D12/PC6  2 UART6 RX */
    {&gpioc, &timer8,NULL,  7, TIMER_CH2, nADC}, /* D13/PC7  3 UART6 TX */
    {&gpioc, &timer8,NULL,  8, TIMER_CH3, nADC}, /* D14/PC8  4 SERVO 5 */
    {&gpioc, &timer8,NULL,  9, TIMER_CH4, nADC}, /* D15/PC9  5 SERVO_6 */
    {&gpioc,   NULL, NULL, 10, NO_CH,     nADC}, /* D16/PC10 6 UART3 RX */
    {&gpioc,   NULL, NULL, 11, NO_CH,     nADC}, /* D17/PC11 7 UART3 TX  */
    {&gpioc,   NULL, NULL, 12, NO_CH,     nADC}, /* D18/PC12 8 UART5_TX */
    {&gpioc,   NULL, NULL, 13, NO_CH,     nADC}, /* D19/PC13 9 USB_SENSE */
    {&gpioc,   NULL, NULL, 14, NO_CH,     nADC}, /* D20/PC14 20 SDCARD_CS */
    {&gpioc,   NULL, NULL, 15, NO_CH,     nADC}, /* D21/PC15 1 buzzer */
    {&gpioa, &timer1,NULL,  8, TIMER_CH1, nADC}, /* D22/PA8  2 SERVO_9 */
    {&gpioa, &timer1,NULL,  9, TIMER_CH2, nADC}, /* D23/PA9  3 USART1_RX */
    {&gpioa, &timer1,NULL, 10, TIMER_CH3, nADC}, /* D24/PA10 4 USART1_TX */
    {&gpiob, &timer4,NULL,  9, TIMER_CH4, nADC}, /* D25/PB9  5 I2C1 SDA */
    {&gpiod,   NULL, NULL,  2, NO_CH,     nADC}, /* D26/PD2  6 UART5_RX */
    {&gpiod,   NULL, NULL,  3, NO_CH,     nADC}, /* D27/PD3  7 */
    {&gpiod,   NULL, NULL,  6, NO_CH,     nADC}, /* D28/PD6  8 */
    {&gpiog,   NULL, NULL, 11, NO_CH,     nADC}, /* D29/PG11 9 */
    {&gpiog,   NULL, NULL, 12, NO_CH,     nADC}, /* D30/PG12 30*/
    {&gpiog,   NULL, NULL, 13, NO_CH,     nADC}, /* D31/PG13 1*/
    {&gpiog,   NULL, NULL, 14, NO_CH,     nADC}, /* D32/PG14 2*/
    {&gpiog,   NULL, NULL,  8, NO_CH,     nADC}, /* D33/PG8  3*/
    {&gpiog,   NULL, NULL,  7, NO_CH,     nADC}, /* D34/PG7  4*/
    {&gpiog,   NULL, NULL,  6, NO_CH,     nADC}, /* D35/PG6  5*/
    {&gpiob, &timer3,NULL,  5, TIMER_CH2, nADC}, /* D36/PB5  6  SPI3 MOSI */
    {&gpiob, &timer4,NULL,  6, TIMER_CH1, nADC}, /* D37/PB6  7  SERVO 2 */
    {&gpiob, &timer4,NULL,  7, TIMER_CH2, nADC}, /* D38/PB7  8  SERVO 1 */
    {&gpiof,   NULL,&_adc3, 6, NO_CH,        4}, /* D39/PF6  9*/
    {&gpiof,   NULL,&_adc3, 7, NO_CH,        5}, /* D40/PF7  40*/
    {&gpiof,   NULL,&_adc3, 8, NO_CH,        6}, /* D41/PF8  1*/
    {&gpiof,   NULL,&_adc3, 9, NO_CH,        7}, /* D42/PF9  2*/
    {&gpiof,   NULL,&_adc3,10, NO_CH,        8}, /* D43/PF10 3*/
    {&gpiof,   NULL, NULL, 11, NO_CH,     nADC}, /* D44/PF11 4*/
    {&gpiob, &timer3,&_adc1,1, TIMER_CH4,    9}, /* D45/PB1  5  SERVO 4 */
    {&gpiob, &timer3,&_adc1,0, TIMER_CH3,    8}, /* D46/PB0  6  SERVO 3 */
    {&gpioa, &timer2,&_adc1,0, TIMER_CH1,    0}, /* D47/PA0  7  UART4 TX (wkup)*/
    {&gpioa, &timer2,&_adc1,1, TIMER_CH2,    1}, /* D48/PA1  8  UART4_RX */
    {&gpioa, &timer2,&_adc1,2, TIMER_CH3,    2}, /* D49/PA2  9  PPM_IN 2 */
    {&gpioa, &timer2,&_adc1,3, TIMER_CH4,    3}, /* D50/PA3  50 PPM_IN 1 */
    {&gpioa,   NULL, &_adc1,4, NO_CH,        4}, /* D51/PA4  1  MPU6000_CS */
    {&gpioa,   NULL, &_adc1,5, NO_CH,        5}, /* D52/PA5  2  SPI1_CLK */
    {&gpioa, &timer3,&_adc1,6, TIMER_CH1,    6}, /* D53/PA6  3  SPI1_MISO */
    {&gpioa, &timer3,&_adc1,7, TIMER_CH2,    7}, /* D54/PA7  4  SPI1_MOSI */
    {&gpiof,   NULL, NULL,  0, NO_CH,     nADC}, /* D55/PF0  5*/
    {&gpiod,   NULL, NULL, 11, NO_CH,     nADC}, /* D56/PD11 6*/
    {&gpiod, &timer4,NULL, 14, TIMER_CH3, nADC}, /* D57/PD14 7*/
    {&gpiof,   NULL, NULL,  1, NO_CH,     nADC}, /* D58/PF1  8*/
    {&gpiod, &timer4,NULL, 12, TIMER_CH1, nADC}, /* D59/PD12 9*/
    {&gpiod, &timer4,NULL, 15, TIMER_CH4, nADC}, /* D60/PD15 60*/
    {&gpiof,   NULL, NULL,  2, NO_CH,     nADC}, /* D61/PF2  1*/
    {&gpiod, &timer4,NULL, 13, TIMER_CH2, nADC}, /* D62/PD13 2*/
    {&gpiod,   NULL, NULL,  0, NO_CH,     nADC}, /* D63/PD0  3*/
    {&gpiof,   NULL, NULL,  3, NO_CH,     nADC}, /* D64/PF3  4*/
    {&gpioe,   NULL, NULL,  3, NO_CH,     nADC}, /* D65/PE3  5*/
    {&gpiod,   NULL, NULL,  1, NO_CH,     nADC}, /* D66/PD1  6*/
    {&gpiof,   NULL, NULL,  4, NO_CH,     nADC}, /* D67/PF4  7*/
    {&gpioe,   NULL, NULL,  4, NO_CH,     nADC}, /* D68/PE4  8*/
    {&gpioe,   NULL, NULL,  7, NO_CH,     nADC}, /* D69/PE7  9*/
    {&gpiof,   NULL, NULL,  5, NO_CH,     nADC}, /* D70/PF5  70*/
    {&gpioe,   NULL, NULL,  5, NO_CH,     nADC}, /* D71/PE5  1*/
    {&gpioe,   NULL, NULL,  8, NO_CH,     nADC}, /* D72/PE8  2*/
    {&gpiof,   NULL, NULL, 12, NO_CH,     nADC}, /* D73/PF12 3*/
    {&gpioe,   NULL, NULL,  6, NO_CH,     nADC}, /* D74/PE6  4*/
    {&gpioe, &timer1,NULL,  9, TIMER_CH1, nADC}, /* D75/PE9  */
    {&gpiof,   NULL, NULL, 13, NO_CH,     nADC}, /* D76/PF13 6*/
    {&gpioe,   NULL, NULL, 10, NO_CH,     nADC}, /* D77/PE10 7*/
    {&gpiof,   NULL, NULL, 14, NO_CH,     nADC}, /* D78/PF14 8*/
    {&gpiog,   NULL, NULL,  9, NO_CH,     nADC}, /* D79/PG9  9*/
    {&gpioe, &timer1,NULL, 11, TIMER_CH2, nADC}, /* D80/PE11 */
    {&gpiof,   NULL, NULL, 15, NO_CH,     nADC}, /* D81/PF15 1*/
    {&gpiog,   NULL, NULL, 10, NO_CH,     nADC}, /* D82/PG10 2*/
    {&gpioe,   NULL, NULL, 12, NO_CH,     nADC}, /* D83/PE12 3*/
    {&gpiog,   NULL, NULL,  0, NO_CH,     nADC}, /* D84/PG0  4*/
    {&gpiod,   NULL, NULL,  5, NO_CH,     nADC}, /* D85/PD5  5*/
    {&gpioe, &timer1,NULL, 13, TIMER_CH3, nADC}, /* D86/PE13 */
    {&gpiog,   NULL, NULL,  1, NO_CH,     nADC}, /* D87/PG1  7*/
    {&gpiod,   NULL, NULL,  4, NO_CH,     nADC}, /* D88/PD4  8*/
    {&gpioe, &timer1,NULL, 14, TIMER_CH4, nADC}, /* D89/PE14 */
    {&gpiog,   NULL, NULL,  2, NO_CH,     nADC}, /* D90/PG2  90*/
    {&gpioe,   NULL, NULL,  1, NO_CH,     nADC}, /* D91/PE1  1*/
    {&gpioe,   NULL, NULL, 15, NO_CH,     nADC}, /* D92/PE15 2*/
    {&gpiog,   NULL, NULL,  3, NO_CH,     nADC}, /* D93/PG3  3*/
    {&gpioe,   NULL, NULL,  0, NO_CH,     nADC}, /* D94/PE0  4*/
    {&gpiod,   NULL, NULL,  8, NO_CH,     nADC}, /* D95/PD8  5*/
    {&gpiog,   NULL, NULL,  4, NO_CH,     nADC}, /* D96/PG4  6*/
    {&gpiod,   NULL, NULL,  9, NO_CH,     nADC}, /* D97/PD9  7*/
    {&gpiog,   NULL, NULL,  5, NO_CH,     nADC}, /* D98/PG5  8*/
    {&gpiod,   NULL, NULL, 10, NO_CH,     nADC}, /* D99/PD10 9*/
    {&gpiob,   NULL, NULL, 11, NO_CH,     nADC}, /* D100/PB11 100  I2C2_SDA */
    {&gpiob, &timer4,NULL,  8, TIMER_CH3, nADC}, /* D101/PB8  I2C1 SCL  */
    {&gpioe,   NULL, NULL,  2, NO_CH,     nADC}, /* D102/PE2 */
    {&gpioa, &timer2,NULL, 15, TIMER_CH1, nADC}, /* D103/PA15 LED_STRIP */
    {&gpiob,   NULL, NULL,  3, NO_CH,     nADC}, /* D104/PB3  SPI3 SCK */
    {&gpiob,   NULL, NULL,  4, NO_CH,     nADC}, /* D105/PB4  SPI3 MISO */
    {&gpioa,   NULL, NULL, 13, NO_CH,     nADC}, /* D106/PA13 SWDIO  GREEN_LED */
    {&gpioa,   NULL, NULL, 14, NO_CH,     nADC}, /* D107/PA14 SWCLK  BLUE_LED */
    {&gpioa,   NULL, NULL, 11, NO_CH,     nADC}, /* D108/PA11  - USB D- */
    
};


extern const struct TIM_Channel PWM_Channels[] __FLASH__ =   {
//CH1 and CH2 also for PPMSUM / SBUS / DSM
    { // 0 RC_IN1
	.pin         = PA3, // UART2 RX
    },
    { // 1 RC_IN2
	.pin         = PA2, // UART2 TX
    },
};


extern const SPIDesc spi_device_table[] = {    // different SPI tables per board subtype
//               name            device   bus  mode         cs_pin                 speed_low       speed_high   dma               priority           delay_cs_on delay_cs_off
     { BOARD_INS_MPU60x0_NAME,   _SPI1,   1,  SPI_MODE_0, BOARD_MPU6000_CS_PIN,    SPI_1_125MHZ,   SPI_9MHZ,   SPI_TRANSFER_DMA, DMA_Priority_VeryHigh,   1,          5 },
     { BOARD_OSD_NAME,           _SPI2,   2,  SPI_MODE_0, BOARD_OSD_CS_PIN,        SPI_1_125MHZ,   SPI_4_5MHZ, SPI_TRANSFER_DMA, DMA_Priority_Low,        2,          2 },
     { BOARD_SDCARD_NAME,        _SPI3,   3,  SPI_MODE_3, 255,                     SPI_1_125MHZ,   SPI_18MHZ,  SPI_TRANSFER_DMA, DMA_Priority_Medium,     2,          2 },
};

extern const uint8_t F4Light_SPI_DEVICE_NUM_DEVICES = ARRAY_SIZE(spi_device_table);

void boardInit(void) {


#ifdef BOARD_HMC5883_DRDY_PIN
    // Init HMC5883 DRDY EXT_INT pin - but it not used by driver
    gpio_set_mode(PIN_MAP[BOARD_HMC5883_DRDY_PIN].gpio_device, PIN_MAP[BOARD_HMC5883_DRDY_PIN].gpio_bit, GPIO_INPUT_PU);
#endif

#ifdef BOARD_MPU6000_DRDY_PIN
    // Init MPU6000 DRDY pin - but it not used by driver
    gpio_set_mode(PIN_MAP[BOARD_MPU6000_DRDY_PIN].gpio_device, PIN_MAP[BOARD_MPU6000_DRDY_PIN].gpio_bit, GPIO_INPUT_PU);
#endif

#ifdef BOARD_SBUS_INVERTER
// it is not necessary because of 10K resistor to ground
    gpio_set_mode( PIN_MAP[BOARD_SBUS_INVERTER].gpio_device, PIN_MAP[BOARD_SBUS_INVERTER].gpio_bit, GPIO_OUTPUT_PP);
    gpio_write_bit(PIN_MAP[BOARD_SBUS_INVERTER].gpio_device, PIN_MAP[BOARD_SBUS_INVERTER].gpio_bit, 0); // not inverted
#endif

}


#endif
