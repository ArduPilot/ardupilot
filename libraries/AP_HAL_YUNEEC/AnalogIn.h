
#ifndef __AP_HAL_YUNEEC_ANALOGIN_H__
#define __AP_HAL_YUNEEC_ANALOGIN_H__

#include <AP_HAL_YUNEEC.h>
#include <utility/pinmap_typedef.h>
#include <stm32f37x.h>
#include <stm32f37x_dma.h>

#define YUNEEC_INPUT_MAX_CHANNELS 		16
#define YUNEEC_VCC_ANALOG_IN_PIN 		PC5

typedef void (*voidFuncPtr)(void);

class YUNEEC::YUNEECAnalogSource : public AP_HAL::AnalogSource {
public:
    friend class YUNEEC::YUNEECAnalogIn;

    YUNEECAnalogSource(uint8_t pin);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();

private:
    // what pin it is attached to
    uint8_t _pin;
    uint8_t _stop_pin;
    uint8_t _stop_pin_high;
    uint16_t _settle_time_ms;
    uint32_t _read_start_time_ms;

    // what value it has
    volatile uint8_t _sum_count;
    volatile uint16_t _sum;
    volatile uint16_t _latest;
    float _last_average;

//    int8_t _pin_scaling_id;
//    static const uint8_t _num_pin_scaling;

    static uint8_t _ADCChannels_Tab[YUNEEC_INPUT_MAX_CHANNELS];
    static uint8_t _num_adc_channels;

    volatile static uint16_t _ADCConvData_Tab[YUNEEC_INPUT_MAX_CHANNELS];
    int8_t _channel_rank;
    uint16_t _get_conv_data(void);

    void new_sample(uint16_t);

//    float _pin_scaler();
    static void _init_adc1(void);
    static void _update_adc1_config();
    void _register_adc_channel(uint8_t pin);
    void _unregister_adc_channel();
};

class YUNEEC::YUNEECAnalogIn : public AP_HAL::AnalogIn {
public:
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t pin);
    float board_voltage(void);
    // servo rail voltage in volts, or 0 if unknown
    float servorail_voltage(void) { return 0; }
    // power supply status flags, see MAV_POWER_STATUS
    uint16_t power_status_flags(void) { return 0; }

private:
    static YUNEECAnalogSource* _channels[YUNEEC_INPUT_MAX_CHANNELS];
    static uint8_t _num_channels;
    static uint8_t _current_stop_pin_i;

    static void _next_stop_pin(void);
    static void _dma_event(void);
    static void _attach_interrupt(voidFuncPtr callback);
};
#endif // __AP_HAL_YUNEEC_ANALOGIN_H__
