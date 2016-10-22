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
#include "AP_Baro_QURT.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

extern const AP_HAL::HAL &hal;

AP_Baro_QURT::AP_Baro_QURT(AP_Baro &baro)
    : AP_Baro_Backend(baro)
{
    instance = _frontend.register_sensor();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Baro_QURT::timer, void));
}

/*
  transfer data to the frontend
 */
void AP_Baro_QURT::update(void)
{
    if (count == 0) {
        return;
    }

    if (lock.take(1)) {
        float temperature = temp_sum / count;
        float pressure = press_sum / count;

        count = 0;
        temp_sum = 0;
        press_sum = 0;
        
        _copy_to_frontend(instance, pressure, temperature);
        lock.give();
    }
}

void AP_Baro_QURT::timer(void)
{
    if (handle == 0) {
        int ret = bmp280_open("/dev/i2c-2", &handle);        
        if (ret != 0) {
            AP_HAL::panic("unable to open QURT baro");
            return;
        }
        last_timer_ms = AP_HAL::millis();
        return;
    }
    if (AP_HAL::millis() - last_timer_ms < 10) {
        return;
    }
    if (lock.take(1)) {
        last_timer_ms = AP_HAL::millis();
        struct bmp280_sensor_data data;
        int ret = bmp280_get_sensor_data(handle, &data, false);
        if (ret != 0 || data.sensor_read_counter == last_counter) {
            lock.give();
            return;
        }
        last_counter = data.sensor_read_counter;
        temp_sum += data.temperature_in_c;
        press_sum += data.pressure_in_pa;
        count++;
        lock.give();
    }
}

#endif // CONFIG_HAL_BOARD
