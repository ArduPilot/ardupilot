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

#define BOARD_BUZZER_PIN        105 
#define HAL_BUZZER_ON           1
#define HAL_BUZZER_OFF          0

#define BOARD_NR_USARTS         5
#define BOARD_USART1_TX_PIN     23 
#define BOARD_USART1_RX_PIN     24 
#define BOARD_USART3_TX_PIN     0
#define BOARD_USART3_RX_PIN     100
#define BOARD_USART6_TX_PIN     12
#define BOARD_USART6_RX_PIN     13

#define BOARD_USART4_RX_PIN     48
#define BOARD_USART4_TX_PIN     47

//#define BOARD_USART5_RX_PIN     26  // PD2  EXTI_RFM22B / UART5_RX
//#define BOARD_BUTTON_PIN        103 // PA15 CS_RFM22B


#define BOARD_DSM_USART (_USART1)
   
#define BOARD_NR_SPI            3
#define BOARD_SPI1_SCK_PIN      52
#define BOARD_SPI1_MISO_PIN     53
#define BOARD_SPI1_MOSI_PIN     54
#define BOARD_SPI2_SCK_PIN      3 // PB13
#define BOARD_SPI2_MISO_PIN     4 // PB14
#define BOARD_SPI2_MOSI_PIN     5 // PB15
#define BOARD_SPI3_MOSI_PIN     18
#define BOARD_SPI3_MISO_PIN     17
#define BOARD_SPI3_SCK_PIN      16



#define BOARD_MPU6000_CS_PIN	51
#define BOARD_MPU6000_DRDY_PIN	10  // PC4


//#define BOARD_SBUS_INVERTER     6

#define BOARD_USB_SENSE 11      // PC5


// bus 2 (soft) pins
#define BOARD_SOFT_SCL 14
#define BOARD_SOFT_SDA 15

// SoftSerial pins
//#define BOARD_SOFTSERIAL_TX 14
//#define BOARD_SOFTSERIAL_RX 15


# define BOARD_BLUE_LED_PIN        36  // BLUE
# define BOARD_GREEN_LED_PIN        6   // NC - PC0 pin 8

# define HAL_GPIO_A_LED_PIN      BOARD_BLUE_LED_PIN
# define HAL_GPIO_B_LED_PIN      BOARD_GREEN_LED_PIN

# define HAL_GPIO_LED_ON           LOW
# define HAL_GPIO_LED_OFF          HIGH


#define BOARD_NR_GPIO_PINS      109

//TODO add #define BOARD_HAS_UART3 ?


#define I2C1_SDA PB9
#define I2C1_SCL PB8

#define I2C2_SDA PB11
#define I2C2_SCL PB10


//#define BOARD_I2C_BUS_INT 1    // hardware internal I2C
#define BOARD_I2C_BUS_EXT 1  // external I2C
#define BOARD_I2C_BUS_SLOW 1   // slow down bus with this number

#define BOARD_I2C1_DISABLE // lots of drivers tries to scan all buses, spoiling device setup

#define BOARD_BARO_DEFAULT HAL_BARO_BMP280_SPI
#define HAL_BARO_BMP280_NAME "bmp280"
#define BOARD_BMP280_CS_PIN 104

#define HAL_BARO_MS5611_I2C_BUS         BOARD_I2C_BUS_EXT
#define HAL_BARO_MS5611_I2C_ADDR        (0x77)

#define BOARD_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#define BOARD_COMPASS_HMC5843_I2C_ADDR 0x1E
#define BOARD_COMPASS_HMC5843_ROTATION ROTATION_NONE

#define HAL_COMPASS_HMC5843_I2C_BUS     BOARD_I2C_BUS_EXT
#define HAL_COMPASS_HMC5843_I2C_ADDR    BOARD_COMPASS_HMC5843_I2C_ADDR
#define HAL_COMPASS_HMC5843_ROTATION    BOARD_COMPASS_HMC5843_ROTATION


#define BOARD_INS_DEFAULT HAL_INS_MPU60XX_SPI
#define BOARD_INS_ROTATION  ROTATION_YAW_180
#define BOARD_INS_MPU60x0_NAME            "mpu6000"

#define BOARD_STORAGE_SIZE            8192 // 4096 // EEPROM size


#define BOARD_SDCARD_NAME "sdcard"
#define BOARD_SDCARD_CS_PIN    2
#define BOARD_SDCARD_DET_PIN  38 // PB7

#define BOARD_HAS_SDIO
#define HAL_BOARD_LOG_DIRECTORY "0:/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "0:/APM/TERRAIN"
//#define HAL_PARAM_DEFAULTS_PATH "0:/APM/defaults.parm"
#define USB_MASSSTORAGE 

#define BOARD_OSD_NAME       "osd"
#define BOARD_OSD_CS_PIN     103
#define BOARD_OSD_VSYNC_PIN   9 // PC3, Frequency input
#define BOARD_OSD_RESET_PIN   6 // PC0, NC


#define BOARD_OWN_NAME "AirbotV2"

# define BOARD_PUSHBUTTON_PIN   254
# define BOARD_USB_MUX_PIN      -1
# define BOARD_BATTERY_VOLT_PIN     8   // Battery voltage on A0 (PC2) D8
# define BOARD_BATTERY_CURR_PIN     7   // Battery current on A1 (PC1) D7
# define BOARD_SONAR_SOURCE_ANALOG_PIN 254

# define HAL_BATT_VOLT_PIN      8 // ChibiOS compatible defines
# define HAL_BATT_CURR_PIN      8
# define HAL_BATT_VOLT_SCALE    10.1
# define HAL_BATT_CURR_SCALE    17


#define BOARD_USB_DMINUS 108

#define BOARD_SBUS_UART 1 // can use some UART as hardware inverted input

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define BOARD_UARTS_LAYOUT 3

// use soft I2C driver instead hardware
//#define BOARD_SOFT_I2C

#define SERVO_PIN_1 46 // PB0 
#define SERVO_PIN_2 45 // PB1
#define SERVO_PIN_3 50 // PA3
#define SERVO_PIN_4 49 // PA2
#define SERVO_PIN_5 48 // PA1
#define SERVO_PIN_6 22 // PA8

#define MOTOR_LAYOUT_DEFAULT 0

#if 1
 #define HAL_CONSOLE USB_Driver // console on USB
 #define HAL_CONSOLE_PORT 0
#else
 #define HAL_CONSOLE uart1Driver // console on radio
 #define HAL_CONSOLE_PORT 1
#endif

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


