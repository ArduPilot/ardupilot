#ifndef _BOARD_STM32V1F4_H_
#define _BOARD_STM32V1F4_H_


/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
 */
#define __CM4_REV                 0x0001  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present      */

#define HSE_VALUE (8000000)

#define CYCLES_PER_MICROSECOND  (SystemCoreClock / 1000000)
#define SYSTICK_RELOAD_VAL      (CYCLES_PER_MICROSECOND*1000-1)

#undef  STM32_PCLK1
#undef  STM32_PCLK2
#define STM32_PCLK1   (CYCLES_PER_MICROSECOND*1000000/4)
#define STM32_PCLK2   (CYCLES_PER_MICROSECOND*1000000/2)

#define BOARD_BUTTON_PIN     254

#ifndef LOW
# define LOW 0
#endif
#ifndef HIGH 
# define HIGH 1
#endif

#define BOARD_BUZZER_PIN        PC15
#define HAL_BUZZER_ON     0
#define HAL_BUZZER_OFF    1

#define BOARD_NR_USARTS         4

#define BOARD_HAS_UART3

#define BOARD_USART1_TX_PIN     PA10
#define BOARD_USART1_RX_PIN     PA9

//#define BOARD_USART2_TX_PIN     PA2
//#define BOARD_USART2_RX_PIN     PA3 - used for PPM

#define BOARD_USART3_TX_PIN     PC11
#define BOARD_USART3_RX_PIN     PC10

#define BOARD_USART4_TX_PIN     PA0
#define BOARD_USART4_RX_PIN     PA1

#define BOARD_USART5_RX_PIN     PD2
#define BOARD_USART5_TX_PIN     PC12

#define BOARD_USART6_TX_PIN     PC6
#define BOARD_USART6_RX_PIN     PC7
   
#define BOARD_NR_SPI            3
#define BOARD_SPI1_SCK_PIN      PA5
#define BOARD_SPI1_MISO_PIN     PA6
#define BOARD_SPI1_MOSI_PIN     PA7

#define BOARD_SPI2_SCK_PIN      PB13
#define BOARD_SPI2_MISO_PIN     PC2
#define BOARD_SPI2_MOSI_PIN     PC3

#define BOARD_SPI3_SCK_PIN      PB3
#define BOARD_SPI3_MISO_PIN     PB4
#define BOARD_SPI3_MOSI_PIN     PB5


#define BOARD_MPU6000_CS_PIN	PA4
#define BOARD_MPU6000_DRDY_PIN	PC4


#define BOARD_USB_SENSE PC13

#define I2C1_SDA PB9
#define I2C1_SCL PB8

#define I2C2_SDA PB11
#define I2C2_SCL PB10

// bus 2 (soft) pins
//#define BOARD_SOFT_SCL 14
//#define BOARD_SOFT_SDA 15

// SoftSerial pins
//#define BOARD_SOFTSERIAL_TX 14
//#define BOARD_SOFTSERIAL_RX 15


//#define BOARD_BLUE_LED_PIN        PA14
//#define BOARD_GREEN_LED_PIN       PA13

//#define HAL_GPIO_A_LED_PIN      BOARD_BLUE_LED_PIN
//#define HAL_GPIO_B_LED_PIN      BOARD_GREEN_LED_PIN

# define BOARD_LED_ON           LOW
# define BOARD_LED_OFF          HIGH
# define HAL_GPIO_LED_ON        LOW
# define HAL_GPIO_LED_OFF       HIGH


#define BOARD_NR_GPIO_PINS      109

#define BOARD_I2C_BUS_INT 0    // hardware internal I2C
#define BOARD_I2C_BUS_EXT 1     // external I2C
#define BOARD_I2C_BUS_SLOW 1    // slow down bus with this number

#define BOARD_BARO_DEFAULT              HAL_BARO_BMP280_I2C
#define HAL_BARO_BMP280_BUS             BOARD_I2C_BUS_INT
#define HAL_BARO_BMP280_I2C_ADDR        (0x76)
#define HAL_BARO_BMP280_I2C_ADDR_ALT    (0x77)

#define HAL_BARO_MS5611_I2C_BUS         BOARD_I2C_BUS_EXT
#define HAL_BARO_MS5611_I2C_ADDR        (0x77)

#define BOARD_COMPASS_DEFAULT           HAL_COMPASS_HMC5843
#define BOARD_COMPASS_HMC5843_I2C_ADDR  0x1E
#define BOARD_COMPASS_HMC5843_ROTATION  ROTATION_NONE

#define HAL_COMPASS_HMC5843_I2C_BUS     BOARD_I2C_BUS_EXT
#define HAL_COMPASS_HMC5843_I2C_ADDR    BOARD_COMPASS_HMC5843_I2C_ADDR
#define HAL_COMPASS_HMC5843_ROTATION    BOARD_COMPASS_HMC5843_ROTATION

#define BOARD_INS_DEFAULT               HAL_INS_MPU60XX_SPI
#define BOARD_INS_ROTATION              ROTATION_YAW_270
#define BOARD_INS_MPU60x0_NAME          "mpu6000"

#define BOARD_STORAGE_SIZE              8192 // 4096 // EEPROM size


#define BOARD_SDCARD_NAME       "sdcard"
#define BOARD_SDCARD_CS_PIN     PC14
//#define BOARD_SDCARD_DET_PIN  38 // PB7

#define BOARD_HAS_SDIO
#define HAL_BOARD_LOG_DIRECTORY "0:/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "0:/APM/TERRAIN"
//#define HAL_PARAM_DEFAULTS_PATH "0:/APM/defaults.parm"
#define USB_MASSSTORAGE 

#define BOARD_OSD_NAME          "osd"
#define BOARD_OSD_CS_PIN        PB12
//#define BOARD_OSD_VSYNC_PIN   9 
//#define BOARD_OSD_RESET_PIN   6 


/*
#define DATAFLASH_CS_PIN           PC0
#define BOARD_DATAFLASH_NAME       "dataflash"
#define BOARD_DATAFLASH_PAGES      0x10000
#define BOARD_DATAFLASH_ERASE_SIZE (4096)// in bytes
*/

#define BOARD_OWN_NAME "MatekF4-wing"

# define BOARD_PUSHBUTTON_PIN   254
# define BOARD_USB_MUX_PIN      -1
# define BOARD_BATTERY_VOLT_PIN     PC0 // Battery voltage 
# define BOARD_BATTERY_CURR_PIN     PC1    // Battery current 
# define BOARD_SONAR_SOURCE_ANALOG_PIN PC5 // rssi 

# define HAL_BATT_VOLT_PIN      PC5 // ChibiOS compatible defines
# define HAL_BATT_CURR_PIN      PC4
# define HAL_BATT_VOLT_SCALE    10.1
# define HAL_BATT_CURR_SCALE    31.7

#define BOARD_USB_DMINUS 108

//#define BOARD_SBUS_INVERTER     
//#define BOARD_SBUS_UART 2 // can use some UART as hardware inverted input

#define LED_STRIP_PIN           PA15

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define BOARD_UARTS_LAYOUT 6

#define SERVO_PIN_1 PB7  // S1
#define SERVO_PIN_2 PB6  // S2
#define SERVO_PIN_3 PB0  // S3
#define SERVO_PIN_4 PB1  // S4
#define SERVO_PIN_5 PC8  // S5
#define SERVO_PIN_6 PC9  // S6
#define SERVO_PIN_7 PB14 // S7
#define SERVO_PIN_8 PB15 // S8
#define SERVO_PIN_9 PA8  // S9

#define MOTOR_LAYOUT_DEFAULT 0 // Ardupilot

#define HAL_CONSOLE USB_Driver // console on USB
#define HAL_CONSOLE_PORT 0

/*

    // @Param: USB_STORAGE
    // @DisplayName: allows access to SD card at next reboot
    // @Description: Allows to read/write internal SD card via USB mass-storage protocol. Auto-reset.
    // @Values: 0:normal, 1:work as USB flash drive
    // @User: Advanced
    AP_GROUPINFO("USB_STORAGE",  8, AP_Param_Helper, _usb_storage, 0), \

    // @Param: SD_REFORMAT
    // @DisplayName: Allows to re-format SD card in case of errors in FS
    // @Description: Any FS errors that cause failure of logging will be corrected by SD card formatting
    // @Values: 0: not allow, 1:allow
    // @User: Advanced
    AP_GROUPINFO("SD_REFORMAT",     7, AP_Param_Helper, _sd_format, 0),

*/
#define BOARD_HAL_VARINFO \
    AP_GROUPINFO("USB_STORAGE",  30, AP_Param_Helper, _usb_storage, 0), \
    AP_GROUPINFO("SD_REFORMAT",  31, AP_Param_Helper, _sd_format, 0),
    

// parameters
#define BOARD_HAL_PARAMS \
    AP_Int8 _usb_storage; \
    AP_Int8 _sd_format; 
#endif


