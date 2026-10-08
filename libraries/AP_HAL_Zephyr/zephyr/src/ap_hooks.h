/*
 * C-callable functions the C++ HAL provides to the Zephyr-side C sources.
 * One declaration, included by the definition and by every caller, so the
 * two cannot drift and -Wmissing-declarations holds - the role
 * AP_HAL_ChibiOS/hwdef/common/stm32_util.h plays for ChibiOS.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Scheduler.cpp: records a fault for the next boot's persistent data, called
   from k_sys_fatal_error_handler() in ap_fault_handler.c. */
void ap_persistent_save_fault(uint16_t line, uint8_t fault_type,
                              uint32_t fault_addr, uint32_t fault_lr,
                              uint32_t fault_icsr);

/* ap_fault_handler.c: hand back the fault record that survived the reset, so
   the HAL can report it over MAVLink. Returns false when nothing is stored.
   Clears the record, so it is reported once per reset. */
bool ap_fault_record_take(unsigned int *reason, uint32_t *pc, uint32_t *lr,
                          uint32_t *cfsr, uint32_t *icsr, uint32_t *thd_prio,
                          uint32_t *count);

/* ap_fault_handler.c: read the surviving fault record WITHOUT clearing it, so
   restore_persistent_data() can fill the fields AP's repeating WDG statustext
   prints and the monitor thread can still report it. */
bool ap_fault_record_peek(unsigned int *reason, uint32_t *pc, uint32_t *lr,
                          uint32_t *cfsr, uint32_t *icsr, uint32_t *thd_prio,
                          uint32_t *count);

/* ap_fault_handler.c: the watchdog stall record, written from the hardware
   watchdog's pre-reset interrupt in Scheduler.cpp. A watchdog reset records no
   fault, so this is the only account of what the main loop was waiting on.
   Cleared by take(), so it is reported once per reset. */
void ap_wdg_record_put(uint32_t stall_ms, int32_t sched_task, uint32_t cur_prio,
                       const char *cur_name, uint32_t main_state,
                       uint32_t main_pended, uint32_t main_pc);
bool ap_wdg_record_take(uint32_t *stall_ms, int32_t *sched_task,
                        uint32_t *cur_prio, char *cur_name, size_t cur_name_len,
                        uint32_t *main_state, uint32_t *main_pended,
                        uint32_t *main_pc);

/* Scheduler.cpp: stamp flash-operation state into the persistent record from
   the ROM flash path itself. op: 1 erase, 2 program. in_flight: operations
   started minus completed, so 1 while one is running and 0 when it finished. */
void ap_persistent_flash_mark(uint32_t op, uint32_t offset, uint32_t in_flight);

/* Util.cpp: renders @SYS/threads.txt into g_ap_sysinfo so it can be read over
   SWD when MAVFTP will not serve it. Called from the io thread. */
void ap_sysinfo_capture(void);

/* ap_pcprofile.c: statistical PC profiler sampled from the system clock
   interrupt. Called from the monitor thread's 10 s report; prints the region
   split (ITCM / OCRAM / external NOR) and the hottest 64-byte buckets. */
void ap_pcprofile_report(void);

#ifdef __cplusplus
}
#endif
