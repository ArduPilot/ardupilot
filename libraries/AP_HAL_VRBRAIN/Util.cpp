
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
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
#include <systemlib/board_serial.h>
#include <drivers/drv_gpio.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"
using namespace VRBRAIN;

extern bool _vrbrain_thread_should_exit;

/*
  constructor
 */
VRBRAINUtil::VRBRAINUtil(void) : Util()
{
    _safety_handle = orb_subscribe(ORB_ID(safety));
}


/*
  start an instance of nsh
 */
bool VRBRAINUtil::run_debug_shell(AP_HAL::BetterStream *stream)
{
	VRBRAINUARTDriver *uart = (VRBRAINUARTDriver *)stream;
    int fd;

    // trigger exit in the other threads. This stops use of the
    // various driver handles, and especially the px4io handle,
    // which otherwise would cause a crash if px4io is stopped in
    // the shell
    _vrbrain_thread_should_exit = true;
    
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
    
    nsh_consolemain(0, nullptr);
    
    // this shouldn't happen
    hal.console->printf("shell exited\n");
    return true;
}

/*
  return state of safety switch
 */
enum VRBRAINUtil::safety_state VRBRAINUtil::safety_switch_state(void)
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

void VRBRAINUtil::set_system_clock(uint64_t time_utc_usec)
{
    timespec ts;
    ts.tv_sec = time_utc_usec/1000000ULL;
    ts.tv_nsec = (time_utc_usec % 1000000ULL) * 1000UL;
    clock_settime(CLOCK_REALTIME, &ts);    
}

/*
  display VRBRAIN system identifer - board type and serial number
 */
bool VRBRAINUtil::get_system_id(char buf[40])
{
    uint8_t serialid[12];
    memset(serialid, 0, sizeof(serialid));
    get_board_serial(serialid);
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
    const char *board_type = "VRBRAINv45";
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
    const char *board_type = "VRBRAINv51";
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V52)
    const char *board_type = "VRBRAINv52";
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
    const char *board_type = "VRUBRAINv51";
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)
    const char *board_type = "VRUBRAINv52";
#elif defined(CONFIG_ARCH_BOARD_VRCORE_V10)
    const char *board_type = "VRCOREv10";
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V54)
    const char *board_type = "VRBRAINv54";
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
uint32_t VRBRAINUtil::available_memory(void)
{
    return mallinfo().fordblks;
}

/*
  AP_HAL wrapper around PX4 perf counters
 */
VRBRAINUtil::perf_counter_t VRBRAINUtil::perf_alloc(VRBRAINUtil::perf_counter_type t, const char *name)
{
    ::perf_counter_type vrbrain_t;
    switch (t) {
    case VRBRAINUtil::PC_COUNT:
        vrbrain_t = ::PC_COUNT;
        break;
    case VRBRAINUtil::PC_ELAPSED:
        vrbrain_t = ::PC_ELAPSED;
        break;
    case VRBRAINUtil::PC_INTERVAL:
        vrbrain_t = ::PC_INTERVAL;
        break;
    default:
        return nullptr;
    }
    return (perf_counter_t)::perf_alloc(vrbrain_t, name);
}

void VRBRAINUtil::perf_begin(perf_counter_t h)
{
    ::perf_begin((::perf_counter_t)h);
}

void VRBRAINUtil::perf_end(perf_counter_t h)
{
    ::perf_end((::perf_counter_t)h);
}

void VRBRAINUtil::perf_count(perf_counter_t h)
{
    ::perf_count((::perf_counter_t)h);
}

void VRBRAINUtil::set_imu_temp(float current)
{
    if (!_heater.target || *_heater.target == -1) {
        return;
    }

    // average over temperatures to remove noise
    _heater.count++;
    _heater.sum += current;
    
    // update once a second
    uint32_t now = AP_HAL::millis();
    if (now - _heater.last_update_ms < 1000) {
        return;
    }
    _heater.last_update_ms = now;

    current = _heater.sum / _heater.count;
    _heater.sum = 0;
    _heater.count = 0;

    // experimentally tweaked for Pixhawk2
    const float kI = 0.3f;
    const float kP = 200.0f;
    
    float err = ((float)*_heater.target) - current;

    _heater.integrator += kI * err;
    _heater.integrator = constrain_float(_heater.integrator, 0, 70);

    float output = constrain_float(kP * err + _heater.integrator, 0, 100);
    
    // hal.console->printf("integrator %.1f out=%.1f temp=%.2f err=%.2f\n", _heater.integrator, output, current, err);

    if (_heater.fd == -1) {
        _heater.fd = open("/dev/px4io", O_RDWR);
    }
    if (_heater.fd != -1) {
        ioctl(_heater.fd, GPIO_SET_HEATER_DUTY_CYCLE, (unsigned)output);
    }
   
}

void VRBRAINUtil::set_imu_target_temp(int8_t *target)
{
    _heater.target = target;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
