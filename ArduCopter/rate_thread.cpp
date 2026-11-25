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
#include "Copter.h"
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>
#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED

/*
  Copter's part of AP_Vehicle's rate thread, see libraries/AP_Vehicle/rate_thread.cpp
 */

// log the rate loop at 1kHz, or the main loop rate if the rate loop is slower, with ATTITUDE_FAST, or at 10Hz with ATTITUDE_MED
AP_Vehicle::RateThreadLogging Copter::get_rate_thread_logging()
{
#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        return RateThreadLogging::FAST;
    }
    if (should_log(MASK_LOG_ATTITUDE_MED)) {
        return RateThreadLogging::MEDIUM;
    }
#endif
    return RateThreadLogging::NONE;
}

/*
  log only those items that are updated at the rate loop rate
 */
void Copter::rate_thread_log_update()
{
#if HAL_LOGGING_ENABLED
    if (!copter.flightmode->logs_attitude()) {
        Log_Write_Rate();
        Log_Write_PIDS(); // only logs if PIDS bitmask is set
    }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    if (should_log(MASK_LOG_FTN_FAST)) {
        AP::ins().write_notch_log_messages();
    }
#endif
#endif
}

// run notch update at either loop rate or 200Hz
void Copter::update_dynamic_notch_at_specified_rate_main()
{
    if (using_rate_thread()) {
        return;
    }

    update_dynamic_notch_at_specified_rate();
}

#endif // AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
