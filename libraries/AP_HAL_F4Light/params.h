

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
    AP_GROUPINFO("SERVO_MASK", 2, AP_Param_Helper, _servo_mask, 0) \

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
    
    // @Param: RC_INPUT
    // @DisplayName: Type of RC input
    // @Description: allows to force specified RC input port
    // @Values: 0:auto, 1:PPM1 (pin3), 2: PPM2 (pin4) etc
    // @User: Advanced
    AP_GROUPINFO("RC_INPUT",     9, AP_Param_Helper, _rc_input, 0)

    // @Param: AIBAO_FS
    // @DisplayName: Support FailSafe for Walkera Aibao RC
    // @Description: Allows to translate of  Walkera Aibao RC FailSafe to Ardupilot's failsafe
    // @Values: 0: not translate, 1:translate
    // @User: Advanced
    AP_GROUPINFO("AIBAO_FS",     7, AP_Param_Helper, _aibao_fs, 0)

    // @Param: RC_FS
    // @DisplayName: Set time of RC failsafe
    // @Description: if none of RC channel changes in that time, then failsafe triggers
    // @Values: 0: turned off, >0 - time in seconds. Good values are starting 60s for digital protocols
    // @User: Advanced
    AP_GROUPINFO("RC_FS",        17, AP_Param_Helper, _rc_fs, 0)

*/

// common parameters for all boards
#define F4LIGHT_HAL_VARINFO \
    AP_GROUPINFO("MOTOR_LAYOUT", 1, AP_Param_Helper, _motor_layout, MOTOR_LAYOUT_DEFAULT), \
    AP_GROUPINFO("SOFTSERIAL",   2, AP_Param_Helper, _use_softserial, 0), \
    AP_GROUPINFO("UART_SBUS",    3, AP_Param_Helper, _uart_sbus, 0), \
    AP_GROUPINFO("SERVO_MASK",   4, AP_Param_Helper, _servo_mask, 0), \
    AP_GROUPINFO("CONNECT_COM",  5, AP_Param_Helper, _connect_com, 0), \
    AP_GROUPINFO("PWM_TYPE",     7, AP_Param_Helper, _pwm_type, 0), \
    AP_GROUPINFO("CONNECT_ESC",  6, AP_Param_Helper, _connect_esc, 0), \
    AP_GROUPINFO("TIME_OFFSET",  8, AP_Param_Helper, _time_offset, 0), \
    AP_GROUPINFO("CONSOLE_UART", 9, AP_Param_Helper, _console_uart, HAL_CONSOLE_PORT), \
    AP_GROUPINFO("EE_DEFERRED",  10, AP_Param_Helper, _eeprom_deferred, 0), \
    AP_GROUPINFO("RC_INPUT",     11, AP_Param_Helper, _rc_input, 0), \
    AP_GROUPINFO("AIBAO_FS",     12, AP_Param_Helper, _aibao_fs, 0), \
    AP_GROUPINFO("OVERCLOCK",    13, AP_Param_Helper, _overclock, 0), \
    AP_GROUPINFO("CORRECT_GYRO", 14, AP_Param_Helper, _correct_gyro, 0), \
    AP_GROUPINFO("RC_FS",        15, AP_Param_Helper, _rc_fs, 0),      \
    AP_GROUPINFO("BOOT_DFU",     16, AP_Param_Helper, _boot_dfu, 0),


// parameters
#define F4LIGHT_HAL_PARAMS \
    AP_Int8 _motor_layout; \
    AP_Int8 _uart_sbus; \
    AP_Int8 _use_softserial; \
    AP_Int8 _servo_mask; \
    AP_Int8 _connect_com;  \
    AP_Int8 _connect_esc; \
    AP_Int8 _pwm_type; \
    AP_Int8 _time_offset; \
    AP_Int8 _console_uart; \
    AP_Int8 _eeprom_deferred; \
    AP_Int8 _rc_input; \
    AP_Int8 _aibao_fs; \
    AP_Int8 _overclock; \
    AP_Int8 _correct_gyro; \
    AP_Int8 _rc_fs; \
    AP_Int8 _boot_dfu;
