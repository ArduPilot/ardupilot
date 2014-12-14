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
LinuxDycoLEDStripDriver LinuxUtil::_ledstrip;
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
void LinuxUtil::led_init(uint8_t length)
{
    _ledstrip.init(length);
}

void LinuxUtil::led_set_solid_color(uint8_t led_num, uint8_t color)
{
    _ledstrip.set_solid_color(led_num,color);
}

void LinuxUtil::led_set_pattern(uint16_t led_num,uint16_t color_series[],float bright_series[],
                                uint16_t time_series[],uint8_t res, uint8_t step_cnt)
{
    _ledstrip.set_pattern(led_num,color_series,bright_series,time_series,res,step_cnt);
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

void LinuxUtil::_led_timer_tick()
{
    _ledstrip.generate_beat_pattern();
}
#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX
