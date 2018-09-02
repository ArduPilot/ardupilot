#ifndef _BOARD_STM32V1F4_H_
#define _BOARD_STM32V1F4_H_


#define BOARD_OWN_NAME "F4Light"

/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
 */
#define __CM4_REV               0x0001  /*!< Core revision r0p1                            */
#define __MPU_PRESENT           1       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS        4       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig  0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT           1       /*!< FPU present      */

#define HSE_VALUE (8000000)

#define CYCLES_PER_MICROSECOND  168
#define SYSTICK_RELOAD_VAL      (CYCLES_PER_MICROSECOND*1000-1)

#undef  STM32_PCLK1
#undef  STM32_PCLK2
#define STM32_PCLK1   (CYCLES_PER_MICROSECOND*1000000/4)
#define STM32_PCLK2   (CYCLES_PER_MICROSECOND*1000000/2)

#ifndef LOW
# define LOW 0
#endif
#ifndef HIGH 
# define HIGH 1
#endif

#define BOARD_BUTTON_PIN        254 // no button

//#define BOARD_RFM22B_CS_PIN     103 // PA15 CS_RFM22B
//#define BOARD_RFM22B_INT_PIN    26  // PD2  INT_RFM22
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

#define BOARD_DATAFLASH_CS_PIN  104

#define BOARD_MPU6000_CS_PIN	51
#define BOARD_MPU6000_DRDY_PIN	10  // PC4


#define BOARD_SBUS_INVERTER     6
#define BOARD_SBUS_UART 1         // can use some UART as hardware inverted input


#define BOARD_USB_SENSE 11      // PC5


// bus 2 (soft) pins
#define BOARD_SOFT_SCL          14
#define BOARD_SOFT_SDA          15

// SoftSerial pins
#define BOARD_SOFTSERIAL_TX     14
#define BOARD_SOFTSERIAL_RX     15


# define BOARD_BLUE_LED_PIN     36  // BLUE
# define BOARD_GREEN_LED_PIN   105 // GREEN 
//# define BOARD_GPIO_C_LED_PIN   37  // PB6 YELLOW OPTIONAL (not included)
//# define BOARD_GPIO_C_LED_PIN   9    //  frequency select - resistor to VCC or ground

# define HAL_GPIO_A_LED_PIN      BOARD_BLUE_LED_PIN
# define HAL_GPIO_B_LED_PIN      BOARD_GREEN_LED_PIN

# define HAL_GPIO_LED_ON           LOW
# define HAL_GPIO_LED_OFF          HIGH


#define BOARD_NR_GPIO_PINS      109

#define I2C1_SDA PB9
#define I2C1_SCL PB8

#define I2C2_SDA PB11
#define I2C2_SCL PB10

#define BOARD_I2C_BUS_INT       0  // hardware I2C

#define BOARD_I2C_BUS_EXT      2  // external soft I2C or flexiPort (by parameter)
#define BOARD_I2C_BUS_SLOW     2  // slow down this bus
 
#define BOARD_HAS_UART3
#define BOARD_I2C_FLEXI 1     // I2C can be on Flexi port



#define BOARD_BARO_DEFAULT              HAL_BARO_MS5611_I2C
#define BOARD_BARO_MS5611_I2C_ADDR      0x77


#define HAL_BARO_MS5611_I2C_BUS         BOARD_I2C_BUS_INT
#define HAL_BARO_MS5611_I2C_ADDR        BOARD_BARO_MS5611_I2C_ADDR

//#define HAL_BARO_MS5611_I2C_BUS_EXT     BOARD_I2C_BUS_EXT  // external baro on soft I2C
//#define HAL_BARO_BMP280_BUS             BOARD_I2C_BUS_EXT  // external baro on soft I2C
//#define HAL_BARO_BMP280_I2C_ADDR        (0x76)

#define BOARD_COMPASS_DEFAULT           HAL_COMPASS_HMC5843
#define BOARD_COMPASS_HMC5843_I2C_ADDR  0x1E
#define BOARD_HMC5883_DRDY_PIN          38  // PB7 - but it not used by driver
#define BOARD_COMPASS_HMC5843_ROTATION  ROTATION_YAW_270  

#define HAL_COMPASS_HMC5843_I2C_BUS     BOARD_I2C_BUS_INT
#define HAL_COMPASS_HMC5843_I2C_EXT_BUS BOARD_I2C_BUS_EXT // external compass on soft I2C
#define HAL_COMPASS_HMC5843_I2C_ADDR    BOARD_COMPASS_HMC5843_I2C_ADDR
#define HAL_COMPASS_HMC5843_ROTATION    BOARD_COMPASS_HMC5843_ROTATION

#define BOARD_INS_DEFAULT               HAL_INS_MPU60XX_SPI
#define BOARD_INS_ROTATION              ROTATION_YAW_180
#define BOARD_INS_MPU60x0_NAME          "mpu6000"

#define BOARD_STORAGE_SIZE              8192 // 4096 // EEPROM size

#define BOARD_DATAFLASH_NAME            "dataflash"
#define BOARD_DATAFLASH_PAGES           0x2000
#define BOARD_DATAFLASH_ERASE_SIZE (65536)// in bytes
//#define BOARD_DATAFLASH_EEPROM 1 // use dataflash as EEPROM - TODO

#define BOARD_SDCARD_NAME "sdcard"
#define BOARD_SDCARD_CS_PIN  103 // PA15 CS_RFM22B
#define BOARD_SDCARD_DET_PIN  26 // PD2  EXTI_RFM22B / UART5_RX

#define USB_MASSSTORAGE
#define HAL_BOARD_LOG_DIRECTORY "0:/"
#define HAL_BOARD_TERRAIN_DIRECTORY "0:/TERRAIN"


#define BOARD_UARTS_LAYOUT 1
#define USE_SOFTSERIAL 1

# define BOARD_PUSHBUTTON_PIN           254 // no pushbutton
# define BOARD_USB_MUX_PIN              -1  // no USB mux
# define BOARD_BATTERY_VOLT_PIN         8   // Battery voltage on A0 (PC2) D8
# define BOARD_BATTERY_CURR_PIN         7   // Battery current on A1 (PC1) D7
# define BOARD_SONAR_SOURCE_ANALOG_PIN  254 // no sonar by default


# define HAL_BATT_VOLT_PIN      8 // ChibiOS compatible defines
# define HAL_BATT_CURR_PIN      7
# define HAL_BATT_VOLT_SCALE    10.1
# define HAL_BATT_CURR_SCALE    17


#define BOARD_USB_DMINUS                108

//#define BOARD_NRF_NAME "nrf24"
//#define BOARD_NRF_CS_PIN 103 // PA15 CS_RFM22B

// motor layouts
#define SERVO_PIN_1 46 // PB0 
#define SERVO_PIN_2 45 // PB1
#define SERVO_PIN_3 50 // PA3
#define SERVO_PIN_4 49 // PA2
#define SERVO_PIN_5 48 // PA1
#define SERVO_PIN_6 47 // PA0

// input pins as servo outputs
#define SERVO_PIN_7    5 // PB15  CH2_IN - PPM2 
#define SERVO_PIN_8   12 // PC6  CH3_IN UART6
#define SERVO_PIN_9   13 // PC7  CH4_IN UART6
#define SERVO_PIN_10  14 // PC8  CH5_IN i2c
#define SERVO_PIN_11  15 // PC9  CH6_IN i2c

#define MOTOR_LAYOUT_DEFAULT 0

#define USE_SERIAL_4WAY_BLHELI_INTERFACE


//#define HAL_CONSOLE USB_Driver // console on USB
//#define HAL_CONSOLE_PORT 0 // USB
#define HAL_CONSOLE uart1Driver // console on radio
#define HAL_CONSOLE_PORT 1 // console on radio

/*
    // @Param: FLEXI_I2C
    // @DisplayName: use FlexiPort as I2C, not USART
    // @Description: Allows to switch FlexiPort usage between USART and I2C modes
    // @Values: 0:USART, 1:I2C
    // @User: Advanced
    AP_GROUPINFO("FLEXI_I2C",    6, AP_Param_Helper, _flexi_i2c, 0) \

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
    AP_GROUPINFO("FLEXI_I2C",    30, AP_Param_Helper, _flexi_i2c, 0),  \
    AP_GROUPINFO("USB_STORAGE",  31, AP_Param_Helper, _usb_storage, 0), \
    AP_GROUPINFO("SD_REFORMAT",  32, AP_Param_Helper, _sd_format, 0),
    

// parameters
#define BOARD_HAL_PARAMS \
    AP_Int8 _flexi_i2c; \
    AP_Int8 _usb_storage; \
    AP_Int8 _sd_format; 
    
#define ERROR_USART _USART1 // main port - telemetry, all panic messages goes there

#endif
