/*
(c) 2017 night_ghost@ykoctpa.ru
 
*/
#pragma GCC optimize ("O2")

#include <AP_HAL/HAL.h>
#include <AP_Param_Helper/AP_Param_Helper.h>

#ifdef BOARD_SBUS_UART

#include <exti.h>
#include <timer.h>
#include <usart.h>
#include "RCInput.h"
#include <pwm_in.h>
#include "sbus.h"
#include "GPIO.h"
#include "ring_buffer_pulse.h"

#include "RC_SBUS_parser.h"

using namespace F4Light;


extern const AP_HAL::HAL& hal;

UARTDriver *SBUS_parser::uartSDriver;

void SBUS_parser::init(uint8_t ch){

    memset((void *)&_val[0],    0, sizeof(_val));
    memset((void *)&sbus,       0, sizeof(sbus));
    
    _last_signal=0;
    _last_change =0;
        
    _ioc = Scheduler::register_io_completion(FUNCTOR_BIND_MEMBER(&SBUS_parser::_io_completion, void));

}


void SBUS_parser::late_init(uint8_t b){

        if(hal_param_helper->_uart_sbus) {
#ifdef BOARD_SBUS_INVERTER
            GPIO::_pinMode(BOARD_SBUS_INVERTER, OUTPUT);
            GPIO::_write(  BOARD_SBUS_INVERTER, HIGH); // do inverse
#endif

            const usart_dev * uart = UARTS[hal_param_helper->_uart_sbus];
            if(uart) {
                uartSDriver = new UARTDriver(uart);
            
                // initialize SBUS UART
                uartSDriver->end();
                uartSDriver->begin(100000, (UART_Parity_Even <<4) | UART_Stop_Bits_2);
                if(uartSDriver->is_initialized() ) {
                    Revo_handler h = { .mp = FUNCTOR_BIND_MEMBER(&SBUS_parser::add_uart_input, void) };
                    uartSDriver->setCallback(h.h);
                }
            } else printf("\nWrong HAL_SBUS_UART selected!");
        }
}


/*
  add some bytes of input in SBUS serial stream format, coping with partial packets - UART input callback
 */
void SBUS_parser::add_uart_input() {
    Scheduler::do_io_completion(_ioc);
}


void SBUS_parser::_io_completion() {

    while(uartSDriver->available()){
        
        // at least 1 byte we have
        const uint8_t frame_size = sizeof(sbus.frame);

        //uint32_t now = systick_uptime();
        uint32_t now = Scheduler::_micros();
        if (now - sbus.last_input_uS > 5000) { // 25 bytes * 13 bits on 100 000 baud takes 3250uS
            // resync based on time
            sbus.partial_frame_count = 0;
        }
        sbus.last_input_uS = now;
    
        if (sbus.partial_frame_count + 1 > frame_size) {
            return; // we can't add bytes to buffer
        }
    

        sbus.frame[sbus.partial_frame_count] = uartSDriver->read();
        sbus.partial_frame_count += 1;

	if (sbus.partial_frame_count == frame_size) {
            sbus.partial_frame_count = 0;
            uint16_t values[18] {};
            uint16_t num_values=0;
            bool sbus_failsafe=false, sbus_frame_drop=false;    
            
            if (sbus_decode(sbus.frame, values, &num_values,
                        &sbus_failsafe, &sbus_frame_drop,
                        F4Light_RC_INPUT_NUM_CHANNELS) &&
                num_values >= F4Light_RC_INPUT_MIN_CHANNELS)
            {
    
                for (uint8_t i=0; i<num_values; i++) {
                    if(_val[i] != values[i]) _last_change = systick_uptime();
                    _val[i] = values[i];
                }
                _channels = num_values;

                if (!sbus_failsafe) {
                    _last_signal = systick_uptime();
                }
            }
            
            sbus.partial_frame_count = 0; // clear count when packet done
        }
    }
}


#endif
