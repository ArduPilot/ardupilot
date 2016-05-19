/*
 * AP_Time.cpp
 */

#include "AP_Time.h"

// get system time in UTC hours, minutes, seconds and milliseconds
void get_time_utc(int32_t &hour, int32_t &min, int32_t &sec, int32_t &ms)
{
     // get time of day in ms
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t time_ms = ((uint64_t)(ts.tv_sec * 1000 + ts.tv_nsec/1000000));

    // separate time into ms, sec, min, hour and days but all expressed in milliseconds
    ms = time_ms % 1000;
    uint32_t sec_ms = (time_ms % (60 * 1000)) - ms;
    uint32_t min_ms = (time_ms % (60 * 60 * 1000)) - sec_ms - ms;
    uint32_t hour_ms = (time_ms % (24 * 60 * 60 * 1000)) - min_ms - sec_ms - ms;

    // convert times as milliseconds into appropriate units
    sec = sec_ms / 1000;
    min = min_ms / (60 * 1000);
    hour = hour_ms / (60 * 60 * 1000);
}

// get milliseconds from now until a UTC time specified in hours, min, seconds and milliseconds
// hour, minute and second values can be ignored by setting to -1
// for example specifying hour=-1, minutes=10 will ignore the hour and return milliseconds until 10 minutes past the next hour
uint32_t get_ms_until_time_utc(int32_t hour, int32_t min, int32_t sec, int32_t ms)
{
    // determine highest value specified (0=none, 1=ms, 2=sec, 3=min, 4=hour)
    int8_t largest_element = 0;
    if (ms != -1) largest_element = 1;
    if (sec != -1) largest_element = 2;
    if (min != -1) largest_element = 3;
    if (hour != -1) largest_element = 4;

    // exit immediately if no time specified
    if (largest_element == 0) {
        return 0;
    }

    // get start_time_ms as h, m, s, ms
    int32_t curr_hour, curr_min, curr_sec, curr_ms;
    get_time_utc(curr_hour, curr_min, curr_sec, curr_ms);
    int32_t total_delay_ms = 0;

    // calculate ms to target
    if (largest_element >= 1) {
        total_delay_ms += ms - curr_ms;
    }
    if (largest_element == 1 && total_delay_ms < 0) {
        total_delay_ms += 1000;
    }

    // calculate sec to target
    if (largest_element >= 2) {
        total_delay_ms += (sec - curr_sec)*1000;
    }
    if (largest_element == 2 && total_delay_ms < 0) {
        total_delay_ms += (60*1000);
    }

    // calculate min to target
    if (largest_element >= 3) {
        total_delay_ms += (min - curr_min)*60*1000;
    }
    if (largest_element == 3 && total_delay_ms < 0) {
        total_delay_ms += (60*60*1000);
    }

    // calculate hours to target
    if (largest_element >= 4) {
        total_delay_ms += (hour - curr_hour)*60*60*1000;
    }
    if (largest_element == 4 && total_delay_ms < 0) {
        total_delay_ms += (24*60*60*1000);
    }

    // total delay in milliseconds
    return (uint32_t)total_delay_ms;
}
