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

//#define BOARD_BUZZER_PIN        5 // PB15, PWM2 - used as PPM2


#define BOARD_NR_USARTS         5
#define BOARD_USART1_TX_PIN     23 
#define BOARD_USART1_RX_PIN     24 
#define BOARD_USART3_TX_PIN     0
#define BOARD_USART3_RX_PIN     100
#define BOARD_USART6_TX_PIN     12
#define BOARD_USART6_RX_PIN     13

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



#define BOARD_MPU6000_CS_PIN		51
#define BOARD_MPU6000_DRDY_PIN	10  // PC4


#define BOARD_USB_SENSE 11      // PC5


# define BOARD_BLUE_LED_PIN        36  // BLUE
# define BOARD_GREEN_LED_PIN        9      //  frequency select - resistor to VCC or ground

# define HAL_GPIO_A_LED_PIN      BOARD_BLUE_LED_PIN
# define HAL_GPIO_B_LED_PIN      BOARD_GREEN_LED_PIN

# define HAL_LED_ON           LOW
# define HAL_LED_OFF          HIGH


#define BOARD_NR_GPIO_PINS      109


#define BOARD_I2C_BUS_INT 1  // hardware internal I2C

#define BOARD_I2C_BUS_EXT      2  // external soft I2C
#define BOARD_I2C_BUS_SLOW     2  // slow down this bus

// bus 2 (soft) pins
#define BOARD_SOFT_SCL          105
#define BOARD_SOFT_SDA          26

//#define BOARD_BARO_DEFAULT   HAL_BARO_BMP085_I2C
//#define HAL_BARO_BMP085_BUS  BOARD_I2C_BUS_EXT

#define BOARD_COMPASS_DEFAULT HAL_COMPASS_HMC5843

#define HAL_COMPASS_HMC5843_I2C_BUS     BOARD_I2C_BUS_EXT
#define HAL_COMPASS_HMC5843_I2C_ADDR    (0x1E)
#define HAL_COMPASS_HMC5843_ROTATION    ROTATION_NONE


#define BOARD_INS_DEFAULT HAL_INS_MPU60XX_SPI
#define BOARD_INS_ROTATION  ROTATION_YAW_180
#define BOARD_INS_MPU60x0_NAME            "mpu6000"


#define BOARD_STORAGE_SIZE            8192 //4096 // EEPROM size

#define BOARD_DATAFLASH_NAME "dataflash"
#define BOARD_DATAFLASH_PAGES 0x2000 // in 256-bytes pages
#define BOARD_DATAFLASH_ERASE_SIZE (4096)// in bytes

#if 1// if board's dataflash supports 4k erases then we can use it as FAT and share it via USB
#define BOARD_DATAFLASH_FATFS
#define BOARD_HAS_SDIO
#define USB_MASSSTORAGE
#define HAL_BOARD_LOG_DIRECTORY "0:"
#define HAL_BOARD_TERRAIN_DIRECTORY "0:/TERRAIN"
//#define HAL_PARAM_DEFAULTS_PATH "0:/APM/defaults.parm"
#else
// old dataflash logs
#endif

#define BOARD_OSD_NAME "osd"
#define BOARD_OSD_CS_PIN   103
#define BOARD_OSD_VSYNC_PIN   9 // PC3, Frequency input (pin 11)

#define BOARD_OWN_NAME "MiniF4_OSD"

# define BOARD_PUSHBUTTON_PIN   254
# define BOARD_USB_MUX_PIN      -1
# define BOARD_BATTERY_VOLT_PIN     8   // Battery voltage on A0 (PC2) D8
# define BOARD_BATTERY_CURR_PIN     7   // Battery current on A1 (PC1) D7
# define BOARD_SONAR_SOURCE_ANALOG_PIN 254

#define BOARD_USB_DMINUS 108


#define BOARD_SBUS_UART 1 // can use some UART as hardware inverted input

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// use soft I2C driver instead hardware
//#define BOARD_SOFT_I2C

// motor layouts
#define SERVO_PIN_1 46 // PB0 
#define SERVO_PIN_2 45 // PB1
#define SERVO_PIN_3 50 // PA3
#define SERVO_PIN_4 49 // PA2
#define SERVO_PIN_5 48 // PA1
#define SERVO_PIN_6 22 // PA8


//#define HAL_CONSOLE uart1Driver // console on radio
#define HAL_CONSOLE USB_Driver // console on USB
#define HAL_CONSOLE_PORT 0 // console on USB



/*

    // @Param: MOTOR_LAYOUT
    // @DisplayName: Motor layout scheme
    // @Description: Selects how motors are numbered
    // @Values: 0:ArduCopter, 1: Ardupilot with pins 2&3 for servos 2:OpenPilot,3:CleanFlight
    // @User: Advanced
    AP_GROUPINFO("_MOTOR_LAYOUT", 0,  HAL_F4Light, _motor_layout, 0),

    // @Param: USE_SOFTSERIAL
    // @DisplayName: Use SoftwareSerial driver
    // @Description: Use SoftwareSerial driver instead SoftwareI2C on Input Port pins 7 & 8
    // @Values: 0:disabled,1:enabled
    // @User: Advanced
    AP_GROUPINFO("_USE_SOFTSERIAL", 1,  HAL_F4Light, _use_softserial, 0),

    // @Param: UART_SBUS
    // @DisplayName: What UART to use as SBUS input
    // @Description: Allows to use any UART as SBUS input
    // @Values: 0:disabled,1:UART1, 2:UART2 etc
    // @User: Advanced
    AP_GROUPINFO("UART_SBUS", 3, AP_Param_Helper, _uart_sbus, 0), \
    
    // @Param: SERVO_MASK
    // @DisplayName: Servo Mask of Input port
    // @Description: Enable selected pins of Input port to be used as Servo Out
    // @Values: 0:disabled,1:enable pin3 (PPM_1), 2: enable pin4 (PPM_2), 4: enable pin5 (UART6_TX) , 8: enable pin6 (UART6_RX), 16: enable pin7, 32: enable pin8
    // @User: Advanced
    AP_GROUPINFO("SERVO_MASK", 2, AP_Param_Helper, _servo_mask, 0) 

    // @Param: RC_INPUT
    // @DisplayName: Type of RC input
    // @Description: allows to force specified RC input port
    // @Values: 0:auto, 1:PPM1 (pin3), 2: PPM2 (pin4) etc
    // @User: Advanced
    AP_GROUPINFO("RC_INPUT",     9, AP_Param_Helper, _rc_input, 0)

    // @Param: CONNECT_COM
    // @DisplayName: connect to COM port
    // @Description: Allows to connect USB to arbitrary UART, thus allowing to configure devices on that UARTs. Auto-reset.
    // @Values: 0:disabled, 1:connect to port 1, 2:connect to port 2, etc
    // @User: Advanced
    AP_GROUPINFO("CONNECT_COM", 2, AP_Param_Helper, _connect_com, 0) \

    // @Param: CONNECT_ESC
    // @DisplayName: connect to ESC inputs via 4wayIf
    // @Description: Allows to connect USB to ESC inputs, thus allowing to configure ESC as on 4-wayIf. Auto-reset.
    // @Values: 0:disabled, 1:connect uartA to ESC, 2: connect uartB to ESC, etc
    // @User: Advanced
    AP_GROUPINFO("CONNECT_ESC", 2, AP_Param_Helper, _connect_esc, 0) \

    // @Param: PWM_TYPE
    // @DisplayName: PWM protocol used
    // @Description: Allows to ignore MOT_PWM_TYPE  param and set PWM protocol independently
    // @Values: 0:use MOT_PWM_TYPE, 1:OneShot 2:OneShot125 3:OneShot42 4:PWM125
    // @User: Advanced
    AP_GROUPINFO("PWM_TYPE",     7, AP_Param_Helper, _pwm_type, 0)

    // @Param: USB_STORAGE
    // @DisplayName: allows access to SD card at next reboot
    // @Description: Allows to read/write internal SD card via USB mass-storage protocol. Auto-reset.
    // @Values: 0:normal, 1:work as USB flash drive
    // @User: Advanced
    AP_GROUPINFO("USB_STORAGE",  8, AP_Param_Helper, _usb_storage, 0), \

    // @Param: RC_FS
    // @DisplayName: Set time of RC failsafe
    // @Description: if none of RC channel changes in that time, then failsafe triggers
    // @Values: 0: turned off, >0 - time in seconds. Good values are starting 60s for digital protocols
    // @User: Advanced
    AP_GROUPINFO("RC_FS",        17, AP_Param_Helper, _rc_fs, 0)
*/
#define BOARD_HAL_VARINFO \
    AP_GROUPINFO("MOTOR_LAYOUT", 1, AP_Param_Helper, _motor_layout, 0), \
    AP_GROUPINFO("SERVO_MASK",   2, AP_Param_Helper, _servo_mask, 0), \
    AP_GROUPINFO("UART_SBUS",    3, AP_Param_Helper, _uart_sbus, 0), \
    AP_GROUPINFO("CONNECT_COM",  5, AP_Param_Helper, _connect_com, 0), \
    AP_GROUPINFO("PWM_TYPE",     6, AP_Param_Helper, _pwm_type, 0), \
    AP_GROUPINFO("CONNECT_ESC",  7, AP_Param_Helper, _connect_esc, 0), \
    AP_GROUPINFO("USB_STORAGE",  8, AP_Param_Helper, _usb_storage, 0), \
    AP_GROUPINFO("TIME_OFFSET",  9, AP_Param_Helper, _time_offset, 0), \
    AP_GROUPINFO("CONSOLE_UART", 10, AP_Param_Helper, _console_uart, HAL_CONSOLE_PORT), \
    AP_GROUPINFO("EE_DEFERRED",  11, AP_Param_Helper, _eeprom_deferred, 0), \
    AP_GROUPINFO("RC_INPUT",     12, AP_Param_Helper, _rc_input, 0), \
    AP_GROUPINFO("AIBAO_FS",     13, AP_Param_Helper, _aibao_fs, 0), \
    AP_GROUPINFO("OVERCLOCK",    14, AP_Param_Helper, _overclock, 0), \
    AP_GROUPINFO("CORRECT_GYRO", 15, AP_Param_Helper, _correct_gyro, 0), \
    AP_GROUPINFO("RC_FS",        16, AP_Param_Helper, _rc_fs, 0)
    

// parameters
#define BOARD_HAL_PARAMS \
    AP_Int8 _motor_layout; \
    AP_Int8 _uart_sbus; \
    AP_Int8 _servo_mask;   \
    AP_Int8 _connect_com;  \
    AP_Int8 _connect_esc; \
    AP_Int8 _pwm_type; \
    AP_Int8 _rc_input; \
    AP_Int8 _time_offset; \
    AP_Int8 _console_uart; \
    AP_Int8 _eeprom_deferred; \
    AP_Int8 _usb_storage; \
    AP_Int8 _aibao_fs; \
    AP_Int8 _overclock; \
    AP_Int8 _correct_gyro; \
    AP_Int8 _rc_fs; 
    
#endif

#define USB_MASSSTORAGE 

