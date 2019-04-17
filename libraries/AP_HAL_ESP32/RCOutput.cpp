/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by David "Buzz" Bussenschutt and others
 *
 */

#include "RCOutput.h"

#include "string.h"


//HINTS: see more about MCPWM peripheral here: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/mcpwm.html
// and example code here: https://github.com/espressif/esp-idf/blob/e7f85f1987aa9479c2dbab638ca83bcaef99be00/examples/peripherals/mcpwm/mcpwm_basic_config/main/mcpwm_basic_config_example.c
// and servo example here: https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_servo_control/main/mcpwm_servo_control_example.c

#include <AP_HAL/AP_HAL.h>

// this is the first/only "group" of 4 GPIO devices
#define LIST_GROUP 0
#define NUM_GROUPS ARRAY_SIZE(pwm_group_list)

#define CHAN_DISABLED 255

using namespace ESP32;

struct RCOutput::pwm_group RCOutput::pwm_group_list[] = { LIST_GROUP };

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate
/**
 * @brief Use this function to calcute pulse width for per degree rotation
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void RCOutput::init()
{
	   printf("RCOutput::init()\n");
	   

	   	// configure all 6 GPIOs the same at the same time... as OUTPUTS
		#define GPIO_BIT_MASK  ((1ULL<<GPIO_NUM_21) | (1ULL<<GPIO_NUM_22) | (1ULL<<GPIO_NUM_25) | (1ULL<<GPIO_NUM_27) | (1ULL<<GPIO_NUM_32) | (1ULL<<GPIO_NUM_33))
		gpio_config_t io_conf;
		io_conf.intr_type = GPIO_INTR_DISABLE;
		io_conf.mode = GPIO_MODE_OUTPUT;
		io_conf.pin_bit_mask = GPIO_BIT_MASK;
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
		gpio_config(&io_conf);


	   	// list of 6 pins we'll be using for PWM out:
		int myints[] = {33,32,21,22,25,27};
		// 0A,0B,1A,1B,2A,2B timers, signals and operators.
		mcpwm_timer_t mytimers[] = {MCPWM_TIMER_0,MCPWM_TIMER_0,MCPWM_TIMER_1,MCPWM_TIMER_1,MCPWM_TIMER_2,MCPWM_TIMER_2};
		mcpwm_io_signals_t mysignals[] = {MCPWM0A,MCPWM0B,MCPWM1A,MCPWM1B,MCPWM2A,MCPWM2B};
		mcpwm_operator_t myops[] = {MCPWM_OPR_A,MCPWM_OPR_B,MCPWM_OPR_A,MCPWM_OPR_B,MCPWM_OPR_A,MCPWM_OPR_B};
		// setup all the timers and signals etc
		for ( int i = 0 ; i < 6; i++) {
			   pwm_out out;
			   out.gpio_num = myints[i];
			   out.unit_num = MCPWM_UNIT_0;
			   out.timer_num = mytimers[i];
			   out.io_signal = mysignals[i];
			   out.op = myops[i];  // MCPWM_OPR_A = 0
			   out.chan =  i ; // 0 = CH_1; // see AP_HAL/RCOutput.h, so 0-5 is CH1,CH2,CH3,CH4,CH5,CH6
			   pwm_group_list[0].out_list[i] = out;
		}

//
//	   // hardcode initialisation of pin 21 to UNIT0, TIMER0, 0A etc
//	   pwm_out out0;
//	   out0.gpio_num = 21;
//	   out0.unit_num = MCPWM_UNIT_0;
//	   out0.timer_num = MCPWM_TIMER_0;
//	   out0.io_signal = MCPWM0A;
//	   out0.op = MCPWM_OPR_A;  // MCPWM_OPR_A = 0
//	   out0.chan = CH_1; // see AP_HAL/RCOutput.h
//	   pwm_group_list[0].out_list[0] = out0;
//
//
//	   // hardcode initialisation of pin 22 to UNIT0, TIMER0, 0B etc
//	   pwm_out out1;
//	   out1.gpio_num = 22;
//	   out1.unit_num = MCPWM_UNIT_0;
//	   out1.timer_num = MCPWM_TIMER_0;
//	   out1.io_signal = MCPWM0B;
//	   out1.op = MCPWM_OPR_B;  // MCPWM_OPR_A = 0
//	   out1.chan = CH_2; // see AP_HAL/RCOutput.h
//	   pwm_group_list[0].out_list[1] = out1;
//
//	   //explicitly disable other channels
//	   pwm_out outdisabled;
//	   outdisabled.gpio_num = 0;
//	   outdisabled.unit_num = (mcpwm_unit_t)0;
//	   outdisabled.timer_num = MCPWM_TIMER_0;
//	   outdisabled.io_signal = (mcpwm_io_signals_t)0;
//	   outdisabled.op = (mcpwm_operator_t)0;
//	   outdisabled.chan = CHAN_DISABLED; // see AP_HAL/RCOutput.h
//	   pwm_group_list[0].out_list[2] = outdisabled;
//	   pwm_group_list[0].out_list[3] = outdisabled;
//	   pwm_group_list[0].out_list[4] = outdisabled;
//	   pwm_group_list[0].out_list[5] = outdisabled;

	   // max of first 6 pwm's in the first MCPWM_UNIT_0

//	    printf("initializing mcpwm gpio...\n");
//		#define GPIO_PWM0A_OUT 32   //Set GPIO as PWM0A - whatever is defined in this slot no worky
//		#define GPIO_PWM0B_OUT 33   //Set GPIO as PWM0B
//		#define GPIO_PWM1A_OUT 27   //Set GPIO as PWM1A
//		#define GPIO_PWM1B_OUT 25   //Set GPIO as PWM1B
//		#define GPIO_PWM2A_OUT 22   //Set GPIO as PWM2A
//		#define GPIO_PWM2B_OUT 21   //Set GPIO as PWM2B
//	    mcpwm_pin_config_t pin_config = {
//	        .mcpwm0a_out_num = GPIO_PWM0A_OUT,
//	        .mcpwm0b_out_num = GPIO_PWM0B_OUT,
//	        .mcpwm1a_out_num = GPIO_PWM1A_OUT,
//	        .mcpwm1b_out_num = GPIO_PWM1B_OUT,
//	        .mcpwm2a_out_num = GPIO_PWM2A_OUT,
//	        .mcpwm2b_out_num = GPIO_PWM2B_OUT
//	    };
//	    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
//
//
//	    //2. initialize mcpwm configuration
//	    printf("Configuring Initial Parameters of mcpwm at 400Hz...\n");
//	    mcpwm_config_t pwm_config;
//	    pwm_config.frequency = 400;    //frequency = 1000Hz
//	    pwm_config.cmpr_a = 60.0;       //duty cycle of PWMxA = 60.0%
//	    pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb = 50.0%
//	    pwm_config.counter_mode = MCPWM_UP_COUNTER;
//	    pwm_config.duty_mode = MCPWM_DUTY_MODE_1; // 1 Active low duty,  i.e. duty cycle proportional to low  time for asymmetric MCPWM
//	    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
//
//	    pwm_config.frequency = 400;     //frequency = 500Hz
//	    pwm_config.cmpr_a = 45.9;       //duty cycle of PWMxA = 45.9%
//	    pwm_config.cmpr_b = 7.0;    //duty cycle of PWMxb = 07.0%
//	    pwm_config.counter_mode = MCPWM_UP_COUNTER;
//	    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
//	    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   //Configure PWM1A & PWM1B with above settings
//
//	    pwm_config.frequency = 400;     //frequency = 400Hz
//	    pwm_config.cmpr_a = 23.2;       //duty cycle of PWMxA = 23.2%
//	    pwm_config.cmpr_b = 97.0;       //duty cycle of PWMxb = 97.0%
//	    pwm_config.counter_mode = MCPWM_DOWN_COUNTER;
//	    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
//	    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);   //Configure PWM2A & PWM2B with above settings


        for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
                pwm_group &group = pwm_group_list[i];
                group.current_mode = MODE_PWM_NORMAL;
                for (uint8_t j = 0; j < 5; j++ ) { // max 6 outputs per PWM UNIT,
                        pwm_out out = group.out_list[j];
                        uint8_t chan = out.chan;
                        if (chan >= 12) //chercher la valeur exacte
                                out.chan = CHAN_DISABLED;
                        if (out.chan != CHAN_DISABLED) {
                                group.ch_mask |= (1U << out.chan);
                                mcpwm_gpio_init(out.unit_num, out.io_signal, out.gpio_num);
                                mcpwm_config_t pwm_config;
                                pwm_config.frequency = 50;    //frequency = 400Hz, which just happens to be 8 channels at 50Hz typical rate
                                pwm_config.cmpr_a = 10;    //duty cycle of PWMxA = 0
                                pwm_config.cmpr_b = 90;    //duty cycle of PWMxb = 0
                                pwm_config.counter_mode = MCPWM_UP_COUNTER;
                                pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
                                mcpwm_init(out.unit_num, out.timer_num, &pwm_config);
                        }
                }
        }
        //set_freq(0xFFFF ^ ((1U << 0) - 1), 500);
#ifdef HAL_GPIO_PIN_SAFETY_IN
        safety_state = AP_HAL::Util::SAFETY_DISARMED;
#endif

}



void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
        for (uint8_t i = 0; i < NUM_GROUPS; i++) {
                pwm_group &group = pwm_group_list[i];
                if (freq_hz > 400 && group.current_mode != MODE_PWM_BRUSHED)
                        freq_hz = 400;
                if ((group.ch_mask & chmask) != 0) {
                        for (int j = 0; j < 4; j++) {
                                //pwm_out out = group.out_list[j];
                                //mcpwm_set_frequency(out.unit_num, out.timer_num, freq_hz);
                        }
                }
        }
}


uint16_t RCOutput::get_freq(uint8_t chan)
{
        for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
                pwm_group &group = pwm_group_list[i];
                for (uint8_t j = 0; j < 4; j++) {
                        pwm_out &out = group.out_list[j];
                        if (out.chan == chan)
                                return mcpwm_get_frequency(out.unit_num, out.timer_num);
                }
        }
        return 0;
}

void RCOutput::enable_ch(uint8_t chan)
{
        for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
                pwm_group &group = pwm_group_list[i];
                for (uint8_t j = 0; j < 4; j++) {
                        pwm_out &out = group.out_list[j];
                        if (out.chan == chan)
                                mcpwm_start(out.unit_num, out.timer_num);
                }
        }
}

void RCOutput::disable_ch(uint8_t chan)
{
        for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
                pwm_group &group = pwm_group_list[i];
                for (uint8_t j = 0; j < 4; j++) {
                        pwm_out &out = group.out_list[j];
                        //if (out.chan == chan)
                             //   mcpwm_stop(out.unit_num, out.timer_num);
                }
        }
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
        if (chan >= max_channels)
                return;
        last_sent[chan] = period_us;
        if (safety_state == AP_HAL::Util::SAFETY_DISARMED)
                period_us = safe_pwm[chan];
        period[chan] = period_us;

        if (!corked)
                push();
//    mcpwm_set_duty_in_us(mcpwm.unit, mcpwm.timer, mcpwm.op, period_us);
}

void RCOutput::cork()
{
        corked = true;
}

void RCOutput::push()
{
        corked = false;
        for (uint8_t i = 0; i < NUM_GROUPS; i++) {
                pwm_group &group = pwm_group_list[i];
                for (uint8_t j = 0; j < 4; j++) {
                        uint8_t chan = group.out_list[j].chan;
                        if (chan == CHAN_DISABLED)
                                continue;
                        uint32_t period_us = period[chan];
                        if (safety_state == AP_HAL::Util::SAFETY_DISARMED)
                                period_us = safe_pwm[chan];

                        if (group.current_mode == MODE_PWM_BRUSHED) {
                                if (period_us <= _esc_pwm_min)
                                        period_us = 0;
                                if (period_us >= _esc_pwm_max)
                                        period_us = _esc_pwm_max - 1;
                        } else if (group.current_mode == MODE_PWM_ONESHOT125){
                                period_us = ((chan / 1000000U) * period_us) / 8U;}

                        //else if (group.current_mode < MODE_PWM_DSHOT150)
                        //        period_us = (chan / 1000000U) * period_us; // sets period_us to zero, should not.

                        pwm_out &out = group.out_list[j];

                 //       cal_pulsewidth = (SERVO_MIN_PULSEWIDTH +
                 //       		(((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));


                        mcpwm_set_duty_in_us(out.unit_num, out.timer_num, out.op, period_us);
                }
        }
}

uint16_t RCOutput::read(uint8_t chan)
{
        if (chan >= max_channels)
                return 0;
        return period[chan];
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
        if (len > max_channels) {
                len = max_channels;
        }

        if (len <= chan_offset) {
                return;
        }

        len -= chan_offset;
        period_us += chan_offset;

        memcpy(period_us, period, len * sizeof(uint16_t));
}

uint16_t RCOutput::read_last_sent(uint8_t chan)
{
        if (chan >= max_channels)
                return 0;
        return last_sent[chan];
}

void RCOutput::read_last_sent(uint16_t *period_us, uint8_t len)
{
        if (len > max_channels)
                len = max_channels;
        for (uint8_t i = 0; i < len; i++) {
                period_us[i] = read_last_sent(i);
        }
}

void RCOutput::force_safety_off(void)
{
        safety_state = AP_HAL::Util::SAFETY_ARMED;
}

bool RCOutput::force_safety_on(void)
{
        safety_state = AP_HAL::Util::SAFETY_DISARMED;
        return true;
}
