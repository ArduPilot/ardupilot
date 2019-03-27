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
 */

#include "SoftSigReaderInt.h"
//#include "hwdef/common/stm32_util.h"

#include "driver/timer.h"

// esp32 timer example code here: https://github.com/espressif/esp-idf/blob/release/v3.2/examples/peripherals/timer_group/main/timer_group_example_main.c

//https://github.com/espressif/esp-idf/blob/57e1b5dad/components/driver/include/driver/timer.h
//#include "driver/timer.h"


#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (3.4179) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (5.78)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload


/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    timer_group_t timer_group;
    timer_idx_t timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;

/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
                                    (uint32_t) (counter_value));
    printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group0_isr(timer_idx_t *para)
{
    timer_idx_t timer_idx = (timer_idx_t) *para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = (timer_group_t)0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1;
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void example_tg0_timer_init(timer_idx_t timer_idx,  bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register((timer_group_t)TIMER_GROUP_0, (timer_idx_t) timer_idx, (void (*)(void*))timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, (intr_handle_data_t**)NULL);
//    timer_isr_register(0, 0, 0,  0, 0, 0);

    timer_start(TIMER_GROUP_0, timer_idx);
}

/*
 * The main task of this example program
 */
static void timer_example_evt_task(void *arg)
{
    while (1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        if (evt.type == TEST_WITHOUT_RELOAD) {
            printf("\n    Example timer without reload\n");
        } else if (evt.type == TEST_WITH_RELOAD) {
            printf("\n    Example timer with auto reload\n");
        } else {
            printf("\n    UNKNOWN EVENT TYPE\n");
        }
        printf("Group[%d], timer[%d] alarm event\n", evt.timer_group, evt.timer_idx);

        /* Print the timer values passed by event */
        printf("------- EVENT TIME --------\n");
        print_timer_counter(evt.timer_counter_value);

        /* Print the timer values as visible by this task */
        printf("-------- TASK TIME --------\n");
        uint64_t task_counter_value;
        timer_get_counter_value(evt.timer_group, evt.timer_idx, &task_counter_value);
        print_timer_counter(task_counter_value);
    }
}




#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

using namespace ESP32;
extern const AP_HAL::HAL& hal;

#if HAL_USE_EICU == TRUE

// #if STM32_EICU_USE_TIM10 || STM32_EICU_USE_TIM11 || STM32_EICU_USE_TIM13 || STM32_EICU_USE_TIM14
// #error "Timers with only one channel are not supported"
// #endif

// singleton instance
SoftSigReaderInt *SoftSigReaderInt::_instance;

SoftSigReaderInt::SoftSigReaderInt()
{
    _instance = this;
}




void SoftSigReaderInt::init()
{


    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    //example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
    example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD,    TIMER_INTERVAL1_SEC); 
    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);


   // example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);

}

void SoftSigReaderInt::_irq_handler()
{
    //eicuchannel_t channel = get_pair_channel(aux_channel);

    pulse_t pulse;



    //pulse.w0 = eicup->tim->CCR[channel];
    //pulse.w1 = eicup->tim->CCR[aux_channel];
    
    _instance->sigbuf.push(pulse);

    //check for missed interrupt 
    //uint32_t mask = (STM32_TIM_SR_CC1OF << channel) | (STM32_TIM_SR_CC1OF << aux_channel);
    //if ((eicup->tim->SR & mask) != 0) {
        //we have missed some pulses
        //try to reset RCProtocol parser by returning invalid value (i.e. 0 width pulse)
        pulse.w0 = 0;
        pulse.w1 = 0;
       // _instance->sigbuf.push(pulse);
        //reset overcapture mask
     //   eicup->tim->SR &= ~mask;
    //}
}
bool SoftSigReaderInt::read(uint32_t &widths0, uint32_t &widths1)
{
    if (sigbuf.available() >= 2) {
        pulse_t pulse;
        if (sigbuf.pop(pulse)) {
            widths0 = uint16_t(pulse.w0 - last_value);
            widths1 = uint16_t(pulse.w1 - pulse.w0);
            last_value = pulse.w1;
            return true;
        }
    }
    return false;
}

#endif // HAL_USE_EICU

#endif //CONFIG_HAL_BOARD == HAL_BOARD_ESP32
