#include <uxr/client/util/time.h>
#include <uxr/client/config.h>
#include <time.h>

#ifdef WIN32
#include <Windows.h>
#elif defined(UCLIENT_PLATFORM_FREERTOS_PLUS_TCP)
#include "FreeRTOS.h"
#include "task.h"
#endif /* ifdef WIN32 */

//==================================================================
//                             PUBLIC
//==================================================================
int64_t uxr_millis(
        void)
{
    return uxr_nanos() / 1000000;
}

int64_t uxr_nanos(
        void)
{
#ifdef WIN32
    SYSTEMTIME epoch_tm = {
        1970, 1, 4, 1, 0, 0, 0, 0
    };
    FILETIME epoch_ft;
    SystemTimeToFileTime(&epoch_tm, &epoch_ft);
    uint64_t epoch_time = (((uint64_t) epoch_ft.dwHighDateTime) << 32) + epoch_ft.dwLowDateTime;

    SYSTEMTIME tm;
    FILETIME ft;
    GetSystemTime(&tm);
    SystemTimeToFileTime(&tm, &ft);
    uint64_t current_time = (((uint64_t) ft.dwHighDateTime) << 32) + ft.dwLowDateTime;

    return (current_time - epoch_time) * 100;
#elif defined(UCLIENT_PLATFORM_FREERTOS_PLUS_TCP)
    TimeOut_t tick_count;

    /* Get the current tick count. */
    vTaskSetTimeOutState(&tick_count);

    /* Pack the current tick count in int64_t. */
    int64_t total_tick = (int64_t) tick_count.xOverflowCount;
#if ( configUSE_16_BIT_TICKS == 1 )    /* Use 16-bit tick type. */
    total_tick <<= 16;
#else  //( configUSE_16_BIT_TICKS == 1 ) /* Use 32-bit tick type. */
    total_tick <<= 32;
#endif // ( configUSE_16_BIT_TICKS == 1 )
    total_tick |= (int64_t) tick_count.xTimeOnEntering;
    return (( total_tick / (int64_t) portTICK_PERIOD_MS ) * 1000000 );
#elif defined(UCLIENT_PLATFORM_ZEPHYR)
    struct timespec ts;
    z_impl_clock_gettime(CLOCK_REALTIME, &ts);
    return (((int64_t)ts.tv_sec) * 1000000000) + ts.tv_nsec;
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (((int64_t)ts.tv_sec) * 1000000000) + ts.tv_nsec;
#endif /* ifdef WIN32 */
}
