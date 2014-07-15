/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Flymaple port by Mike McCauley
 */

#ifndef __AP_HAL_YUNEEC_ANALOGIN_H__
#define __AP_HAL_YUNEEC_ANALOGIN_H__

#include <AP_HAL_YUNEEC.h>

#define YUNEEC_INPUT_MAX_CHANNELS 12

// This is the pin number of the pin connected to VCC
// this is not built in to the YUNEEC board, so you must connect
// this pin to the board 3.3V VCC
#define YUNEEC_VCC_ANALOG_IN_PIN 15

class AP_HAL_YUNEEC::YUNEECAnalogSource : public AP_HAL::AnalogSource {
public:
    friend class AP_HAL_YUNEEC::YUNEECAnalogIn;

    YUNEECAnalogSource(uint8_t pin);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();

    /* implementation specific interface: */

    /* new_sample(): called with value of ADC measurments, from interrput */
    void new_sample(uint16_t);

    /* setup_read(): called to setup ADC registers for next measurment,
     * from interrupt */
    void setup_read();

    /* stop_read(): called to stop device measurement */
    void stop_read();

    /* reading_settled(): called to check if we have read for long enough */
    bool reading_settled();

    /* read_average: called to calculate and clear the internal average.
     * implements read_average(), unscaled. */
    float _read_average();

    int16_t get_pin() { return _pin; };
private:
    /* following three are used from both an interrupt and normal thread */
    volatile uint8_t _sum_count;
    volatile uint16_t _sum;
    volatile uint16_t _latest;
    float _last_average;

    /* _pin designates the ADC input mux for the sample */
    uint8_t _pin;

    /* _stop_pin designates a digital pin to use for
       enabling/disabling the analog device */
    uint8_t _stop_pin;
    uint16_t _settle_time_ms;
    uint32_t _read_start_time_ms;
};

class AP_HAL_YUNEEC::YUNEECAnalogIn : public AP_HAL::AnalogIn {
public:
    YUNEECAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);
    float board_voltage(void);

protected: 
    YUNEECAnalogSource* _create_channel(int16_t num);
    void _register_channel(YUNEECAnalogSource*);
    void _timer_event(void);
    YUNEECAnalogSource* _channels[YUNEEC_INPUT_MAX_CHANNELS];
    int16_t _num_channels;
    int16_t _active_channel;
    uint16_t _channel_repeat_count;
    
private:
    // On YUNEEC, VCC measurement is at pin 20. VCC (=VIN) of 5V is only present
    // if external voltage (not USB) is connected. Also there is a voltage
    // divider (25k/5k) 
    YUNEECAnalogSource _vcc;
};
#endif // __AP_HAL_YUNEEC_ANALOGIN_H__
