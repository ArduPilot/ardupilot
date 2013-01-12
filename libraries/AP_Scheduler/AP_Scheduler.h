/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  main loop scheduler for APM
 *  Author: Andrew Tridgell, January 2013
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

#ifndef AP_SCHEDULER_H
#define AP_SCHEDULER_H

#include <AP_Param.h>

/*
  A task scheduler for APM main loops

  Sketches should call scheduler.init() on startup, then call
  scheduler.tick() at regular intervals (typically every 10ms). 

  To run tasks use scheduler.run(), passing the amount of time that
  the scheduler is allowed to use before it must return
 */

class AP_Scheduler
{
public:
	typedef void (*task_fn_t)(void);

	struct Task {
		task_fn_t function;
		uint16_t interval_ticks;
		uint16_t max_time_micros;
	};

	// initialise scheduler
	void init(const Task *tasks, uint8_t num_tasks);

	// call when one tick has passed
	void tick(void);

	// run the tasks. Call this once per 'tick'. 
	// time_available is the amount of time available to run 
	// tasks in microseconds
	void run(uint16_t time_available);

	// return the number of microseconds available for the current task
	uint16_t time_available_usec(void);

    // return debug parameter
    uint8_t debug(void) { return _debug; }

	static const struct AP_Param::GroupInfo var_info[];

private:
	// used to enable scheduler debugging
	AP_Int8 _debug;
	
	// progmem list of tasks to run
	const struct Task *_tasks;
	
	// number of tasks in _tasks list
	uint8_t _num_tasks;

	// number of 'ticks' that have passed (number of times that
	// tick() has been called
	uint16_t _tick_counter;

	// tick counter at the time we last ran each task
	uint16_t *_last_run;

	// number of microseconds allowed for the current task
	uint16_t _task_time_allowed;

	// the time in microseconds when the task started
	uint32_t _task_time_started;
};

#endif // AP_SCHEDULER_H
