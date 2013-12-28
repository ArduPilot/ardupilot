
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <apps/nsh.h>
#include <fcntl.h>
#include "UARTDriver.h"
#include <uORB/uORB.h>
#include <uORB/topics/safety.h>
#include <systemlib/otp.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"
using namespace PX4;

extern bool _px4_thread_should_exit;

/*
  constructor
 */
PX4Util::PX4Util(void) 
{
    _safety_handle = orb_subscribe(ORB_ID(safety));
}


/*
  start an instance of nsh
 */
bool PX4Util::run_debug_shell(AP_HAL::BetterStream *stream)
{
	PX4UARTDriver *uart = (PX4UARTDriver *)stream;
	int fd;

	// trigger exit in the other threads. This stops use of the
	// various driver handles, and especially the px4io handle,
	// which otherwise would cause a crash if px4io is stopped in
	// the shell
	_px4_thread_should_exit = true;

	// take control of stream fd
	fd = uart->_get_fd();

	// mark it blocking (nsh expects a blocking fd)
        unsigned v;
        v = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, v & ~O_NONBLOCK);	

	// setup the UART on stdin/stdout/stderr
	close(0);
	close(1);
	close(2);
	dup2(fd, 0);
	dup2(fd, 1);
	dup2(fd, 2);

	nsh_consolemain(0, NULL);

	// this shouldn't happen
	hal.console->printf("shell exited\n");
	return true;
}

/*
  return state of safety switch
 */
enum PX4Util::safety_state PX4Util::safety_switch_state(void)
{
    if (_safety_handle == -1) {
        _safety_handle = orb_subscribe(ORB_ID(safety));
    }
    if (_safety_handle == -1) {
        return AP_HAL::Util::SAFETY_NONE;
    }
    struct safety_s safety;
    if (orb_copy(ORB_ID(safety), _safety_handle, &safety) != OK) {
        return AP_HAL::Util::SAFETY_NONE;
    }
    if (!safety.safety_switch_available) {
        return AP_HAL::Util::SAFETY_NONE;
    }
    if (safety.safety_off) {
        return AP_HAL::Util::SAFETY_ARMED;
    }
    return AP_HAL::Util::SAFETY_DISARMED;
}

void PX4Util::set_system_clock(uint64_t time_utc_usec)
{
    timespec ts;
    ts.tv_sec = time_utc_usec/1.0e6;
    ts.tv_nsec = (time_utc_usec % 1000000) * 1000;
    clock_settime(CLOCK_REALTIME, &ts);    
}

/*
  display PX4 system identifer - board type and serial number
 */
bool PX4Util::get_system_id(char buf[40])
{
    uint8_t serialid[12];
    memset(serialid, 0, sizeof(serialid));
    val_read(serialid, (const void *)UDID_START, sizeof(serialid));
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
    const char *board_type = "PX4v1";
#else
    const char *board_type = "PX4v2";
#endif
    // this format is chosen to match the human_readable_serial()
    // function in auth.c
    snprintf(buf, 40, "%s %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
             board_type,
             (unsigned)serialid[0], (unsigned)serialid[1], (unsigned)serialid[2], (unsigned)serialid[3], 
             (unsigned)serialid[4], (unsigned)serialid[5], (unsigned)serialid[6], (unsigned)serialid[7], 
             (unsigned)serialid[8], (unsigned)serialid[9], (unsigned)serialid[10],(unsigned)serialid[11]); 
    return true;
}

/**
   how much free memory do we have in bytes.
*/
uint16_t PX4Util::available_memory(void) 
{
    struct mallinfo mem;
    mem = mallinfo();
    if (mem.fordblks > 0xFFFF) {
        return 0xFFFF;
    }
    return mem.fordblks;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
