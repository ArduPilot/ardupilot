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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include "WheelEncoder_Pulse.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// threshold above noise-level where we'll respect velocity for direction determination
#define VELOCITY_THRESHOLD 0.005


// check if pin has changed and initialise gpio event callback
void AP_WheelEncoder_Pulse::update_pin(uint8_t &pin,
                                            uint8_t new_pin,
                                            uint8_t &pin_value)
{
    if (new_pin == pin) {
        // no change
        return;
    }

    // remove old gpio event callback if present
    if (pin != (uint8_t)-1 &&
        !hal.gpio->detach_interrupt(pin)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: Failed to detach from pin %u", pin);
        // ignore this failure or the user may be stuck
    }

    pin = new_pin;

    // install interrupt handler on rising or falling edge of gpio for pin a
    if (new_pin != (uint8_t)-1) {
        hal.gpio->pinMode(pin, HAL_GPIO_INPUT);
        if (!hal.gpio->attach_interrupt(
                pin,
                FUNCTOR_BIND_MEMBER(&AP_WheelEncoder_Pulse::irq_handler,
                                    void,
                                    uint8_t,
                                    bool,
                                    uint32_t),
                AP_HAL::GPIO::INTERRUPT_BOTH)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: Failed to attach to pin %u", pin);
        }
        pin_value = hal.gpio->read(pin);
    }
}

void AP_WheelEncoder_Pulse::update(void)
{
    /* check if pin changed */
    update_pin(last_pin_a, get_pin_a(), last_pin_a_value);

    // disable interrupts to prevent race with irq_handler
    void *irqstate = hal.scheduler->disable_interrupts_save();

    // copy distance and error count so it is accessible to front end
    copy_state_to_frontend(irq_state.distance_count,
                           irq_state.total_count,
                           irq_state.error_count,
                           irq_state.last_reading_ms);

    // restore interrupts
    hal.scheduler->restore_interrupts(irqstate);
}

void AP_WheelEncoder_Pulse::irq_handler(uint8_t pin,
                                             bool pin_value,
                                             uint32_t timestamp)
{
    // sanity check
    if (last_pin_a == 0) {
        return;
    }

    // record update time
    irq_state.last_reading_ms = timestamp * 1e-3f;

    // value didn't change - we missed an interrupt?
    if(last_pin_a_value == pin_value) {
        irq_state.error_count++;
        return;
    }

    // remember last value
    last_pin_a_value = pin_value;

    // count reading
    irq_state.total_count++;

    // +1 for forward movement, -1 for backwards. Let's decide...
    int32_t increment = 0;

    // // get velocity from AHRS
    // Vector3f velocity;
    // if (!AP::ahrs().get_velocity_NED(velocity)) {
    //     // treat unknown velocity as zero
    //     velocity.zero();
    // }

    // // get forward direction (X-axis of the vehicle body frame, in NED)
    // Matrix3f rot = AP::ahrs().get_rotation_body_to_ned();
    // Vector3f forward_dir = rot.colx();  // body X-axis in NED
    // forward_dir.z = 0;                  // only horizontal
    // forward_dir.normalize();           // ensure unit vector

    // // project velocity onto forward direction to get signed ground speed
    // float signed_speed = velocity.dot(forward_dir) * 100.0f; // cm/s

    // get current throttle
    float steering = 0, throttle = 0;
    if(!AP::vehicle()->get_steering_and_throttle(steering, throttle)) {
        irq_state.error_count++;
        return;
    }

    // // throttle contradicting ground_speed?
    // if((throttle < 0 && ground_speed > 0) || (throttle > 0 && ground_speed < 0)) {
    //     // are we getting pushed or pulled?
    //     hal.console->printf(
    //         "AP_WheelEncoder_Pulse: IMU (%f) & throttle (%f) direction mismatch!\n",
    //         ground_speed, throttle
    //     );
    //     irq_state.error_count++;
    //     return;
    // }

    // use throttle to determine direction by default
    increment = throttle < 0 ? -1 : 1;

    // // use speed to override throttle direction?
    // if(ground_speed != 0) {
    //     increment = ground_speed < 0 ? -1: 1;
    // }

    // hal.console->printf(
    //    "AP_WheelEncoder_Pulse: instance: %d, spd: %f, thr: %f, inc: %d, dist: %d err: %d\n", 
    //    _state.instance, signed_speed, throttle, increment, irq_state.distance_count, irq_state.error_count
    // );

    // register movement
    irq_state.distance_count += increment;
}
