/*
 * ap_scheduler_stub.h
 *
 *  Created on: 28 mai 2014
 *      Author: valentin
 */

#ifndef AP_SCHEDULER_STUB_H_
#define AP_SCHEDULER_STUB_H_


#define MAIN_LOOP_MICROS 10000

class AP_Scheduler
{
public:
    float load_average(uint32_t tick_time_usec) const {return 0.0;}
};

#endif /* AP_SCHEDULER_STUB_H_ */
