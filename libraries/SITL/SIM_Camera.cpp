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
  simple camera simulator class
*/

#include "SIM_Camera.h"

#include <GCS_MAVLink/GCS.h>
#include <SITL/SITL.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo Camera::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Camera Sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the Camera simulation
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, Camera, _enable, 0),

    // @Param: SRV_PIN
    // @DisplayName: Camera trigger servo pin
    // @Description: The pin number that the Camera trigger servo is connected to. (start at 1)
    // @Range: 0 32
    // @User: Advanced
    AP_GROUPINFO("SRV_PIN", 1, Camera, _trigger_servo_pin, -1),

    // @Param: GPIO
    // @DisplayName: Camera trigger GPIO pin
    // @Description: The GPIO pin number used to trigger Camera capture.
    // @Range: -1 15
    // @User: Advanced
    AP_GROUPINFO("GPIO", 2, Camera, _trigger_gpio_pin, -1),

    // @Param: SRV_PWM
    // @DisplayName: Camera trigger PWM threshold
    // @Description: PWM threshold for considering the camera trigger servo active.
    // @Range: 1000 2000
    // @Units: PWM
    // @User: Advanced
    AP_GROUPINFO("SRV_PWM", 3, Camera, _trigger_pwm, 1300),

    AP_GROUPEND
};

/*
  update camera state
 */
void Camera::update(const struct sitl_input &input)
{
    bool servo_triggered = false;
    if (_trigger_servo_pin >= 1 && _trigger_servo_pin <= 32) {
        const int16_t servo_pwm = input.servos[_trigger_servo_pin - 1];
        servo_triggered = servo_pwm >= _trigger_pwm;
    }

    bool gpio_triggered = false;
    const auto *sitl = AP::sitl();
    if (sitl != nullptr && _trigger_gpio_pin >= 0 && _trigger_gpio_pin <= 15) {
        gpio_triggered = (sitl->pin_mask.get() & (1U << _trigger_gpio_pin)) != 0;
    }

    const bool trigger_active = servo_triggered || gpio_triggered;
    if (trigger_active && !_last_trigger_state) {
        _image_count++;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL: Camera image %u", (unsigned)_image_count);
    }

    _last_trigger_state = trigger_active;
}
