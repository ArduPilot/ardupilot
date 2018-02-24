
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <Filter/Filter.h>
#include <Filter/ModeFilter.h>
#include <Filter/AverageFilter.h>
#include <Filter/LowPassFilter2p.h>

#include "joypad_remote.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

JoypadRemote joypadremote;
volatile bool JoypadRemote::_sending = false;

static LowPassFilter2pLong low_pass_filter(800,30);
static LowPassFilter2pLong low_pass_filter2(800,30);

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(JoypadRemote, &joypadremote, func, _interval_ticks, _max_time_micros)

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task JoypadRemote::scheduler_tasks[] = {
    SCHED_TASK(update_sensor,   50,   2000),
    SCHED_TASK(live,             1,   1000),
};

void JoypadRemote::live()
{
    hal.gpio->toggle(13);
}

void JoypadRemote::setup(void)
{
    hal.gpio->pinMode(13, HAL_GPIO_OUTPUT);
    hal.gpio->write(13, 0);

    for (uint8_t i = 0; i < 2; i++) {
        state[i].pin = i;
        _analogsensor[i] = new AnalogSensor(&state[i]);
    }

    for (int i = 22; i < 44; i++){
        hal.gpio->pinMode(i, HAL_GPIO_INPUT);
        hal.gpio->write(i, 1);
    }

    controller_data_buffer1 = get_empty_data_controller();
    controller_data_buffer2 = get_empty_data_controller();
    controller_data = get_empty_data_controller();

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&JoypadRemote::send_data, void));
    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
    nowmicros = AP_HAL::micros();
}

void JoypadRemote::loop(void)
{
    if ((AP_HAL::micros() - nowmicros) > 19990) {
        scheduler.tick();
        scheduler.run(20000);
        nowmicros = AP_HAL::micros();
    }
}

void JoypadRemote::update_sensor(void)
{
    _analogsensor[0]->update();
    _analogsensor[1]->update();

    if (!_sending) {
        hal.scheduler->suspend_timer_procs();
        const uint16_t new_value1 =  state[1].distance_cm;
        const uint16_t new_value2 =  state[0].distance_cm;
        filtered_value1 = (int16_t)low_pass_filter.apply(new_value1);
        filtered_value2 = (int16_t)low_pass_filter2.apply(new_value2);
        set_data();
        hal.scheduler->resume_timer_procs();
    }
}

void JoypadRemote::set_data(void)
{

    for (int i = 0; i < BUTTON_ARRAY_LENGTH; i++) {
        controller_data.button_array[i] = 0;
    }

    for (int i = 22; i < 44; i++){
        controller_data.button_array[(i - 22) / 8] |= (!hal.gpio->read(i)) << ((i - 22) % 8);
    }

    controller_data.left_stick_x = filtered_value1;
    controller_data.left_stick_y = filtered_value2;

    set_controller_data(controller_data);
}

void JoypadRemote::send_data(void)
{
    if (_sending) {
        return;
    }
    _sending = true;

    if (hal.uartA->available() > 0) {
        uint8_t inByte = hal.uartA->read();
        if (inByte < sizeof(data_controller_t)) {
            hal.uartA->write(((uint8_t*)&controller_data_buffer1)[inByte]);
        } else {
            inByte = inByte - sizeof(data_controller_t);
            hal.uartA->write(((uint8_t*)&controller_data_buffer2)[inByte]);
        }
    }
    _sending = false;
}

JoypadRemote::data_controller_t JoypadRemote::get_empty_data_controller(void)
{
    data_controller_t controller_data_empty;
    // Make the buttons zero
    for (int i = 0; i < BUTTON_ARRAY_LENGTH; i++) {
        controller_data_empty.button_array[i] = 0;
    }

    controller_data_empty.dpad_left_on = 0;
    controller_data_empty.dpad_up_on = 0;
    controller_data_empty.dpad_right_on = 0;
    controller_data_empty.dpad_down_on = 0;
    controller_data_empty.dummy = 0;

    // Center the sticks
    controller_data_empty.left_stick_x = 1000;
    controller_data_empty.left_stick_y = 1000;
    controller_data_empty.right_stick_x = 1000;
    controller_data_empty.right_stick_y = 1000;
    controller_data_empty.stick3_x = 1000;
    controller_data_empty.stick3_y = 1000;

    return controller_data_empty;
}

void JoypadRemote::set_controller_data(data_controller_t controller_data_set)
{
    if (!_sending) {
        memcpy(&controller_data_buffer1, &controller_data_set, sizeof(data_controller_t));
    }
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup(void)
{
    joypadremote.setup();
}

void loop(void)
{
    joypadremote.loop();
}

AP_HAL_MAIN();
