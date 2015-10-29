#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"
#include "Heat_Pwm.h"

using namespace Linux;


static int state;
ToneAlarm Util::_toneAlarm;

void Util::init(int argc, char * const *argv) {
    saved_argc = argc;
    saved_argv = argv;

#ifdef HAL_UTILS_HEAT
#if HAL_UTILS_HEAT == HAL_LINUX_HEAT_PWM
    _heat = new Linux::HeatPwm(HAL_LINUX_HEAT_PWM_SYSFS_DIR,
                            HAL_LINUX_HEAT_KP,
                            HAL_LINUX_HEAT_KI,
                            HAL_LINUX_HEAT_PERIOD_NS,
                            HAL_LINUX_HEAT_TARGET_TEMP);
#else
    #error Unrecognized Heat
#endif // #if
#else
    _heat = new Linux::Heat();
#endif // #ifdef
}

void Util::set_imu_temp(float current)
{
    _heat->set_imu_temp(current);
}

/**
   return commandline arguments, if available
*/
void Util::commandline_arguments(uint8_t &argc, char * const *&argv)
{
    argc = saved_argc;
    argv = saved_argv;
}

bool Util::toneAlarm_init()
{
    return _toneAlarm.init();
}

void Util::toneAlarm_set_tune(uint8_t tone)
{
    _toneAlarm.set_tune(tone);
}

void Util::_toneAlarm_timer_tick(){
    if(state == 0){
        state = state + _toneAlarm.init_tune();
    }else if(state == 1){
        state = state + _toneAlarm.set_note();
    }
    if(state == 2){
        state = state + _toneAlarm.play();
    }else if(state == 3){
        state = 1;
    }
    
    if(_toneAlarm.is_tune_comp()){
        state = 0;
    }
    
}

void Util::set_system_clock(uint64_t time_utc_usec)
{
#if CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_NONE
    timespec ts;
    ts.tv_sec = time_utc_usec/1.0e6;
    ts.tv_nsec = (time_utc_usec % 1000000) * 1000;
    clock_settime(CLOCK_REALTIME, &ts);    
#endif    
}

bool Util::is_chardev_node(const char *path)
{
    struct stat st;

    if (!path || lstat(path, &st) < 0)
        return false;

    return S_ISCHR(st.st_mode);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX
