/*******************************************************************************
* rc_time.c
* 
* This is a collection of miscellaneous useful functions that are part of the
* robotics cape library. These do not necessarily interact with hardware.
*******************************************************************************/

#include "roboticscape/roboticscape.h"
#include <time.h>
#include <sys/time.h> // for timeval
#include <errno.h>
#include <unistd.h> // for sysconf
#include <stdint.h> // for uint64_t

/*******************************************************************************
* @ void rc_nanosleep(uint64_t ns)
* 
* A wrapper for the normal UNIX nanosleep function which takes a number of
* nanoseconds instead of a timeval struct. This also handles restarting
* nanosleep with the remaining time in the event that nanosleep is interrupted
* by a signal. There is no upper limit on the time requested.
*******************************************************************************/
void rc_nanosleep(uint64_t ns){
	struct timespec req,rem;
	req.tv_sec = ns/1000000000;
	req.tv_nsec = ns%1000000000;
	// loop untill nanosleep sets an error or finishes successfully
	errno=0; // reset errno to avoid false detection
	while(nanosleep(&req, &rem) && errno==EINTR){
		req.tv_sec = rem.tv_sec;
		req.tv_nsec = rem.tv_nsec;
	}
	return;
}

/*******************************************************************************
* @ void rc_usleep(uint64_t ns)
* 
* The traditional usleep function, however common, is deprecated in linux as it
* uses SIGALARM which interferes with alarm and timer functions. This uses the
* new POSIX standard nanosleep to accomplish the same thing which further
* supports sleeping for lengths longer than 1 second. This also handles
* restarting nanosleep with the remaining time in the event that nanosleep is 
* interrupted by a signal. There is no upper limit on the time requested.
*******************************************************************************/
void rc_usleep(unsigned int us){
	rc_nanosleep(us*1000);
	return;
}

/*******************************************************************************
* @ uint64_t rc_timespec_to_micros(timespec ts)
* 
* Returns a number of microseconds corresponding to a timespec struct.
* Useful because timespec structs are annoying.
*******************************************************************************/
uint64_t rc_timespec_to_micros(struct timespec ts){
	return (((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec)/1000;
}

/*******************************************************************************
* @ uint64_t rc_timespec_to_millis(timespec ts)
* 
* Returns a number of milliseconds corresponding to a timespec struct.
* Useful because timespec structs are annoying.
*******************************************************************************/
uint64_t rc_timespec_to_millis(timespec ts){
	return (((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec)/1000000;
}

/*******************************************************************************
* @ uint64_t rc_timeval_to_micros(timeval tv)
* 
* Returns a number of microseconds corresponding to a timespec struct.
* Useful because timeval structs are annoying.
*******************************************************************************/
uint64_t rc_timeval_to_micros(timeval tv){
	return ((uint64_t)tv.tv_sec*1000000)+tv.tv_usec;
}

/*******************************************************************************
* @ uint64_t rc_timeval_to_millis(timeval ts)
* 
* Returns a number of milliseconds corresponding to a timespec struct.
* Useful because timeval structs are annoying.
*******************************************************************************/
uint64_t rc_timeval_to_millis(timeval tv){
	return (((uint64_t)tv.tv_sec*1000000)+tv.tv_usec)/1000;
}

/*******************************************************************************
* @ uint64_t rc_nanos_since_epoch()
* 
* Returns the number of nanoseconds since epoch using system CLOCK_REALTIME
* This function itself takes about 1100ns to complete at 1ghz under ideal
* circumstances.
*******************************************************************************/
uint64_t rc_nanos_since_epoch(){
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return ((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec;
}

/*******************************************************************************
* @ uint64_t rc_nanos_since_boot()
* 
* Returns the number of nanoseconds since system boot using CLOCK_MONOTONIC
* This function itself takes about 1100ns to complete at 1ghz under ideal
* circumstances.
*******************************************************************************/
uint64_t rc_nanos_since_boot(){
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec;
}

/*******************************************************************************
* @ uint64_t rc_nanos_thread_time()
* 
* Returns the number of nanoseconds from when when the calling thread was
* started in CPU time. This time only increments when the processor is working
* on the calling thread and not when the thread is sleeping. This is usually for
* timing how long blocks of user-code take to execute. This function itself
* takes about 2100ns to complete at 1ghz under ideal circumstances.
*******************************************************************************/
uint64_t rc_nanos_thread_time(){
	struct timespec ts;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
	return ((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec;
}

/*******************************************************************************
* @ timespec rc_timespec_diff(timespec start, timespec end)
* 
* Returns the time difference between two timespec structs as another timespec.
* Convenient for use with nanosleep() function and accurately timed loops.
* Unlike timespec_sub defined in time.h, rc_timespec_diff does not care which
* came first, A or B. A positive difference in time is always returned.
*******************************************************************************/
timespec rc_timespec_diff(timespec A, timespec B){
	timespec temp;
	// check which is greater and flip if necessary
	// For the calculation we want A>B
	if(B.tv_sec>A.tv_sec){
		temp = A;
		A = B;
		B = temp;
	} else if (B.tv_sec==A.tv_sec && B.tv_nsec>A.tv_nsec){
		temp = A;
		A = B;
		B = temp;
	}
	// now calculate the difference
	if((A.tv_nsec-B.tv_nsec)<0) {
		temp.tv_sec = A.tv_sec-B.tv_sec-1;
		temp.tv_nsec = 1000000000L+A.tv_nsec-B.tv_nsec;
	}else{
		temp.tv_sec = A.tv_sec-B.tv_sec;
		temp.tv_nsec = A.tv_nsec-B.tv_nsec;
	}
	return temp;
}

/*******************************************************************************
* @ int rc_timespec_add(timespec* start, double seconds)
* 
* Adds an amount of time in seconds to a timespec struct. The time added is a
* floating point value to make respresenting fractions of a second easier.
* the timespec is passed as a pointer so it can be modified in place.
* Seconds may be negative. 
*******************************************************************************/
void rc_timespec_add(timespec* start, double seconds){
	int s = (int)seconds; // round down by truncation
	start->tv_sec += s;
	start->tv_nsec += (seconds-s)*1000000000;
	
	if (start->tv_nsec>=1000000000){
		start->tv_nsec -= 1000000000;
		start->tv_sec += 1;
	}
	else if (start->tv_nsec<0){
		start->tv_nsec += 1000000000;
		start->tv_sec -= 1;
	}
	
	return;
}


