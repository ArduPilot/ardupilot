/*
 * AP_Time.h
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <time.h>

// get system time in UTC hours, minutes, seconds and milliseconds
void get_time_utc(int32_t &hour, int32_t &min, int32_t &sec, int32_t &ms);

// get milliseconds from now until a UTC time specified in hours, min, seconds and milliseconds
// hour, minute and second values can be ignored by setting to -1
// for example specifying hour=-1, minutes=10 will ignore the hour and return milliseconds until 10 minutes past the next hour
uint32_t get_ms_until_time_utc(int32_t hour, int32_t min, int32_t sec, int32_t ms);
