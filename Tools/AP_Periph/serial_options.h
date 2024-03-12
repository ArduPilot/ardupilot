#pragma once

#ifdef HAL_PERIPH_ENABLE_SERIAL_OPTIONS

#ifndef HAL_UART_NUM_SERIAL_PORTS
#define HAL_UART_NUM_SERIAL_PORTS AP_HAL::HAL::num_serial
#endif

class SerialOptionsDev {
public:
    SerialOptionsDev(void);
    static const struct AP_Param::GroupInfo var_info[];
    AP_Int32 options;
    AP_Int8 rtscts;
};

class SerialOptions {
public:
    friend class AP_Periph_FW;
    SerialOptions(void);
    void init(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    SerialOptionsDev devs[HAL_UART_NUM_SERIAL_PORTS];
};


#endif // HAL_PERIPH_ENABLE_SERIAL_OPTIONS
