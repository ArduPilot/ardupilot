#include "Util.h"
#include "AP_HAL_RP.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/structs/sysinfo.h"
#include "hardware/watchdog.h"
#include "hardware/powman.h"
#include "pico/bootrom.h"
#include <unistd.h>
#include <cstdio>

using namespace RP;

Util::Util() :
    _soft_armed(false)
{}

// Returns the amount of free RAM (Heap).
uint32_t Util::available_memory(void) {
    // These symbols are defined by the linker in the .ld file
    extern char __StackLimit; 
    
    // sbrk(0) returns the current pointer to the end of the allocated heap
    char *heap_end = (char*)sbrk(0);
    
    if (heap_end == (char*)-1) {
        return 0;
    }

    // Calculate the space between the current end of the heap and the beginning of the stack
    // This is the most accurate free memory indicator for Cortex-M
    uint32_t free_ram = (uint32_t)(&__StackLimit - heap_end);
    
    return free_ram;
}

// Creates a new semaphore.
// The ArduPilot HAL expects a pointer to an object that controls critical sections.
/*HAL_Semaphore* Util::new_semaphore(void) {
    return new HAL_Semaphore();
}*/

// Gets a unique ID in raw (binary) form.
bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len) {
    // Create a temporary buffer for the ID (usually 8 bytes)
    uint8_t flash_id[8];
    
    flash_get_unique_id(flash_id);
    
    uint8_t copy_len = (len < 8) ? len : 8;
    memcpy(buf, flash_id, copy_len);
    len = copy_len;
    
    return true;
}

// Gets the unique chip ID (formatted as a string).
// Used to identify logs and connect to GCS.
bool Util::get_system_id(char buf[50]) {
    uint8_t id[8];
    flash_get_unique_id(id);
    
    // Format into a hex string
    for (uint8_t i = 0; i < 8; i++) {
        sprintf(&buf[i * 2], "%02X", id[i]);
    }
    buf[16] = '\0';
    return true;
}

// Returns the time in microseconds since the system started (RTC/Timer).
uint64_t Util::get_hw_rtc() const {
    uint64_t offset_ms = ((uint64_t)powman_hw->scratch[0] << 32) | powman_hw->scratch[1];
    if (offset_ms == 0) {
        return 0;
    }
    uint64_t now_ms = powman_timer_get_ms();
    return (now_ms + offset_ms) * 1000ULL;
#if 0 // RTC exists only for RP2040
    if (!rtc_running()) {
        return 0;
    }
    datetime_t dt;
    if (!rtc_get_datetime(&dt)) {
        return 0;
    }
    // Convert datetime_t (Pico SDK) back to struct tm (C standard)
    struct tm tm_info;
    tm_info.tm_year = dt.year - 1900;
    tm_info.tm_mon  = dt.month - 1;
    tm_info.tm_mday = dt.day;
    tm_info.tm_hour = dt.hour;
    tm_info.tm_min  = dt.min;
    tm_info.tm_sec  = dt.sec;
    tm_info.tm_isdst = -1; // We do not take into account daylight saving time for UTC
    // mktime works with local time, so for UTC we use timegm
    // (in the arm-none-eabi toolchain, the timegm function is usually available)
    time_t t = timegm(&tm_info);
    if (t == (time_t)-1) {
        return 0;
    }
    return (uint64_t)t * 1000000ULL;
#endif
}

// Set the RTC hardware time.
// @param time_utc_usec Time in microseconds UTC (UNIX epoch)
void Util::set_hw_rtc(uint64_t time_utc_usec) {
    uint64_t now_ms = powman_timer_get_ms();
    uint64_t set_ms = time_utc_usec / 1000ULL;
    
    powman_hw->scratch[0] = (uint32_t)((set_ms - now_ms) >> 32);
    powman_hw->scratch[1] = (uint32_t)((set_ms - now_ms) & 0xFFFFFFFF);    
#if 0  // RTC exists only for RP2040
    // Convert microseconds to seconds for the standard C library
    time_t time_sec = (time_t)(time_utc_usec / 1000000ULL);
    struct tm *tm_info = gmtime(&time_sec);

    if (tm_info == nullptr) {
        return;
    }
    // Fill the datetime_t structure for Pico SDK
    datetime_t dt = {
        .year  = (int16_t)(tm_info->tm_year + 1900),
        .month = (int8_t)(tm_info->tm_mon + 1), // tm_mon: 0-11, datetime: 1-12
        .day   = (int8_t)tm_info->tm_mday,
        .dotw  = (int8_t)tm_info->tm_wday,      // 0 = Sunday
        .hour  = (int8_t)tm_info->tm_hour,
        .min   = (int8_t)tm_info->tm_min,
        .sec   = (int8_t)tm_info->tm_sec
    };
    // Initialize and set the RTC
    // rtc_init() is safe to call again
    rtc_init();
    rtc_set_datetime(&dt);
#endif
}

// Reboot to Bootloader mode (for USB firmware).
void Util::reboot(bool hold_in_bootloader) {
    if (hold_in_bootloader) {
        // Specific Pico SDK call to enter USB Mass Storage mode
        reset_usb_boot(0, 0);
    } else {
        watchdog_reboot(0, 0, 0);
    }
}

#if HAL_GCS_ENABLED
// Obtaining information about the reason for the last reset.
bool Util::get_reset_info(uint32_t &reason) {
    // Read Chip Reset registers from RP2350
    // 1 = Power On, 2 = Watchdog, 3 = Software
    reason = watchdog_caused_reboot() ? 2 : 1;
    return true;
}
#endif
