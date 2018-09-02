#ifndef _BOARD_STM32V1F4_H_
#define _BOARD_STM32V1F4_H_

/*
    this file included to *ANY* compilation so shouldn't include anything else
*/


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


//#define BOARD_BUZZER_PIN        5 // PB15, PWM2 - used as PPM2
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

#define BOARD_DSM_USART (_USART1)
   
#define BOARD_NR_SPI            3
#define BOARD_SPI1_SCK_PIN      52
#define BOARD_SPI1_MISO_PIN     53
#define BOARD_SPI1_MOSI_PIN     54
#define BOARD_SPI2_SCK_PIN      255
#define BOARD_SPI2_MISO_PIN     255
#define BOARD_SPI2_MOSI_PIN     255
#define BOARD_SPI3_MOSI_PIN     18
#define BOARD_SPI3_MISO_PIN     17
#define BOARD_SPI3_SCK_PIN      16

#define BOARD_DATAFLASH_CS_PIN   104


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
# define BOARD_GREEN_LED_PIN        9      //  frequency select - resistor to VCC or ground

# define HAL_GPIO_A_LED_PIN      BOARD_BLUE_LED_PIN
# define HAL_GPIO_B_LED_PIN      BOARD_GREEN_LED_PIN

# define HAL_GPIO_LED_ON           LOW
# define HAL_GPIO_LED_OFF          HIGH


#define BOARD_NR_GPIO_PINS      109

#define I2C1_SDA PB9
#define I2C1_SCL PB8

#define I2C2_SDA PB11
#define I2C2_SCL PB10

// use soft I2C driver instead hardware
//#define BOARD_SOFT_I2C2
//#define BOARD_I2C_BUS_INT   1  // hardware internal I2C
#define BOARD_I2C_BUS_EXT   1  // external I2C
#define BOARD_I2C_BUS_SLOW  1 // slow down bus with this number

#define HAL_BARO_MS5611_I2C_BUS         BOARD_I2C_BUS_EXT
#define HAL_BARO_MS5611_I2C_ADDR        (0x77)

#define HAL_BARO_BMP280_BUS             BOARD_I2C_BUS_EXT
#define HAL_BARO_BMP280_I2C_ADDR        (0x76)

#define HAL_BARO_BMP085_BUS             BOARD_I2C_BUS_EXT
#define HAL_BARO_BMP085_I2C_ADDR        (0x77)

#define HAL_BARO_MS5607_I2C_BUS         BOARD_I2C_BUS_EXT
#define HAL_BARO_MS5607_I2C_ADDR        (0x77)


#define BOARD_COMPASS_DEFAULT HAL_COMPASS_HMC5843
//#define BOARD_HMC5883_DRDY_PIN  38  // PB7 - but it not used by driver

#define HAL_COMPASS_HMC5843_I2C_BUS     BOARD_I2C_BUS_EXT
#define HAL_COMPASS_HMC5843_I2C_ADDR    (0x1E)
#define HAL_COMPASS_HMC5843_ROTATION    ROTATION_NONE

#define BOARD_INS_DEFAULT HAL_INS_MPU60XX_SPI
#define BOARD_INS_ROTATION  ROTATION_YAW_180
#define BOARD_INS_MPU60x0_NAME            "mpu6000"

#define BOARD_STORAGE_SIZE            8192 //4096 // EEPROM size


#define BOARD_DATAFLASH_NAME "dataflash"
#define BOARD_DATAFLASH_PAGES 0x10000
#define BOARD_DATAFLASH_ERASE_SIZE (4096)// in bytes

// if board's dataflash supports 4k erases then we can use it as FAT and share it via USB
#define BOARD_DATAFLASH_FATFS
#define BOARD_HAS_SDIO
#define USB_MASSSTORAGE
#define HAL_BOARD_LOG_DIRECTORY "0:"
#define HAL_BOARD_TERRAIN_DIRECTORY "0:/TERRAIN"
//#define HAL_PARAM_DEFAULTS_PATH "0:/defaults.parm"

#define BOARD_UARTS_LAYOUT 2

#define BOARD_OWN_NAME "Airbot"

# define BOARD_PUSHBUTTON_PIN   254
# define BOARD_USB_MUX_PIN      -1
# define BOARD_BATTERY_VOLT_PIN     8   // Battery voltage on A0 (PC2) D8
# define BOARD_BATTERY_CURR_PIN     7   // Battery current on A1 (PC1) D7
# define BOARD_SONAR_SOURCE_ANALOG_PIN 254

# define HAL_BATT_VOLT_PIN      8 // ChibiOS compatible defines
# define HAL_BATT_CURR_PIN      7
# define HAL_BATT_VOLT_SCALE    10.1
# define HAL_BATT_CURR_SCALE    17


#define BOARD_USB_DMINUS 108



#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define BOARD_SBUS_UART 1 // can use some UART as hardware inverted input

// motor layouts
#define SERVO_PIN_1 46 // PB0 
#define SERVO_PIN_2 45 // PB1
#define SERVO_PIN_3 50 // PA3
#define SERVO_PIN_4 49 // PA2
#define SERVO_PIN_5 48 // PA1
#define SERVO_PIN_6 22 // PA8

#define MOTOR_LAYOUT_DEFAULT 0

#define HAL_CONSOLE USB_Driver // console on USB
//#define HAL_CONSOLE uart1Driver // console on radio
#define HAL_CONSOLE_PORT 0



/*


*/

#ifdef USB_MASSSTORAGE

#define BOARD_HAL_VARINFO \
    AP_GROUPINFO("USB_STORAGE",  30, AP_Param_Helper, _usb_storage, 0), \
    AP_GROUPINFO("DBG_WAYBACK",  31, AP_Param_Helper, _dbg_wayback, 0), 

#else

#define BOARD_HAL_VARINFO \
    AP_GROUPINFO("DBG_WAYBACK",  30, AP_Param_Helper, _dbg_wayback, 0), 
#endif


// parameters
#define BOARD_HAL_PARAMS \
    AP_Int8 _dbg_wayback; \
    AP_Int8 _usb_storage; 

#define WAYBACK_DEBUG



#endif

