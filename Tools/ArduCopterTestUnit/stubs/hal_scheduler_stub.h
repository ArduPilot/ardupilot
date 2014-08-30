/*
 * hal_scheduler_stub.h
 *
 *  Created on: 28 mai 2014
 *      Author: valentin
 */

#ifndef HAL_SCHEDULER_STUB_H_
#define HAL_SCHEDULER_STUB_H_

class HAL_Scheduler
{
public:
	uint32_t millis(void) {return 10;}
	void reboot (bool reb) {}
};//HAL_Scheduler

#endif /* HAL_SCHEDULER_STUB_H_ */
