#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"
using namespace Linux;


static int state;
ToneAlarm LinuxUtil::_toneAlarm;
/**
   return commandline arguments, if available
*/
void LinuxUtil::commandline_arguments(uint8_t &argc, char * const *&argv)
{
    argc = saved_argc;
    argv = saved_argv;
}

bool LinuxUtil::toneAlarm_init()
{
    return _toneAlarm.init();
}

void LinuxUtil::toneAlarm_set_tune(uint8_t tone)
{
    _toneAlarm.set_tune(tone);
}

void LinuxUtil::_toneAlarm_timer_tick(){
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
#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX
