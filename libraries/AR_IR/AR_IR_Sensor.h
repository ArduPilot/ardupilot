#pragma once

#define IR_1_PIN_NUM  16
#define IR_2_PIN_NUM  17

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>


class AR_IR_Backend;
class AR_IR_Analog;
class AR_IR_SITL_Analog;
class AR_IR
{
    friend class AR_IR_Backend;
    friend class AR_IR_Analog;
    friend class AR_IR_SITL_Analog;
    
public:
    AR_IR();
    void read();
    /* Do not allow copies */
    AR_IR(const AR_IR &other) = delete;
    AR_IR &operator=(const AR_IR&) = delete;
    void init();
    struct AR_IR_State {
        float voltage_ir_1;
        float voltage_ir_2;
    };
    static const struct AP_Param::GroupInfo var_info[];
    struct {
        AP_Int8 _ir_1_pin_num;  //set pin number for Analog Infrared Sensor 1
        AP_Int8 _ir_2_pin_num;  //set pin number for Analog Infrared Sensor 2
        AP_Float adc_voltage; // adc voltage
    } param;
    void calc_line_error(float &line_error);

private:
    AR_IR_State state;
    AR_IR_Backend *drivers;
};
