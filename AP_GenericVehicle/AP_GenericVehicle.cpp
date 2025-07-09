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
#include "AP_GenericVehicle.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_GenericVehicle genericvehicle;
AP_Vehicle& vehicle = genericvehicle;

AP_GenericVehicle::AP_GenericVehicle()
{
}

#if AP_SCHEDULER_ENABLED
const AP_Scheduler::Task AP_GenericVehicle::scheduler_tasks[] {
#if HAL_GCS_ENABLED
    SCHED_TASK_CLASS(GCS,            (GCS*)&genericvehicle._gcs,       update_receive,   300,  500,  57),
    SCHED_TASK_CLASS(GCS,            (GCS*)&genericvehicle._gcs,       update_send,      300,  750,  60),
#endif  // HAL_GCS_ENABLED
};
void AP_GenericVehicle::get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &task_count, uint32_t &log_bit) {
    tasks = scheduler_tasks;
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = log_bitx;
}
#endif  // AP_SCHEDULER_ENABLED

void AP_GenericVehicle::init_ardupilot()
{
#if AP_INERTIALSENSOR_ENABLED
#if AP_SCHEDULER_ENABLED
    ins.init(scheduler.get_loop_rate_hz());
#else
    ins.init(1000);
#endif  // AP_SCHEDULER_ENABLED
#endif  // AP_INERTIALSENSOR_ENABLED
}

AP_HAL_MAIN_CALLBACKS(&genericvehicle);
