#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "DycoLEDDriver.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>


extern const AP_HAL::HAL& hal;

using namespace Linux;
//RGB values corresponding to     | BLACK |   RED  | ORANGE  |  AMBER  |  YELLOW | GREEN  |  BLUE  | PURPLE  |  WHITE   |
static uint8_t preset_color[9][3]={{0,0,0},{31,0,0},{31,16,0},{31,23,0},{31,31,0},{0,31,0},{0,0,31},{16,0,16},{31,31,31}};
static uint16_t cntr;
LinuxDycoLEDDriver* LinuxDycoLEDStripDriver::_led;

//LED single module functions
LinuxDycoLEDDriver::LinuxDycoLEDDriver()
{    
    data_buf =0x8000;
    clk_prev_time = 0;
    clk_pulse_count = 0;
    data_hold = 0x8000;                      //set the initial color of LED as BLACK 
    _pattern_color = NULL;
    _brightness = NULL;
    _pattern_time = NULL;
}

void LinuxDycoLEDDriver::reset()
{
    data_buf = data_hold;                   //load the latest rgb data sequence for next pop cycle
}

void LinuxDycoLEDDriver::set_rgb(uint16_t red, uint16_t green, uint16_t blue)
{    
    clk_prev_time = 0;
    clk_pulse_count = 0;
    data_hold = red << 10 | green | blue << 5 | 1U<<15;     //1 set_bit(MSB),5 bits Red,5 bits Blue,5 bits Green intensities
                                                            //in the same sequence
}
void LinuxDycoLEDDriver::set_solid_color(uint8_t color)
{
    set_rgb(preset_color[color][0],preset_color[color][1],preset_color[color][2]);
    _res = 0;
}
bool LinuxDycoLEDDriver::pop_data(){

    data_buf = data_buf << 1;
    return (bool)(data_buf & 0x8000);
}
/*
Five Parameters to set pattern:
Color Series: color values after completion or at start of each step of pattern
Time Series: time between color transitions.
Brightness series: Brightness of the color at the same index
Resolution: No. of steps in which color transitions will occur.
Steps: Total no. of color transitions
*/
void LinuxDycoLEDDriver::set_pattern(uint16_t color_series[],float bright_series[],
                                     uint16_t time_series[],uint8_t res, uint8_t step_cnt)
{
    _pattern_color = color_series;
    _brightness = bright_series;
    _pattern_time = time_series;
    _res = res;
    _step_cnt = step_cnt;
}
void LinuxDycoLEDDriver::pattern_step()
{
    int16_t red_diff, blue_diff, green_diff;
    uint16_t red, blue, green;
    uint32_t diff_time;
    uint32_t cur_time = hal.scheduler->millis64();
    uint16_t step_size;
    uint8_t next_color,color;
    if(_res == 0){
        return;                 //pattern not set
    }
    if(_step >= _step_cnt){
        _step = 0;
        _iter = 0;
    }
    
    next_color = _pattern_color[(_step+1)%_step_cnt];
    color = _pattern_color[_step];
    step_size = _pattern_time[_step]/_res;
    diff_time = cur_time - _prev_time;
    
    if((diff_time > step_size) && (_iter < _res)){
        _iter++;
        _prev_time = cur_time;
    }
    //calculate difference between two consecutive colors b/w which transition is taking place
    red_diff = preset_color[next_color][0]*_brightness[(_step+1)%_step_cnt] - preset_color[color][0]*_brightness[_step];
    green_diff = preset_color[next_color][1]*_brightness[(_step+1)%_step_cnt] - preset_color[color][1]*_brightness[_step];
    blue_diff = preset_color[next_color][2]*_brightness[(_step+1)%_step_cnt] - preset_color[color][2]*_brightness[_step];

    //calculate rgb values at current stage/time
    red = preset_color[color][0]*_brightness[_step] + (red_diff*_iter)/_res;
    green = preset_color[color][1]*_brightness[_step] + (green_diff*_iter)/_res;
    blue = preset_color[color][2]*_brightness[_step] + (blue_diff*_iter)/_res;

    if(_iter >= _res){
        _step++;                //get ready for next step/transition
        _iter =0;
    }
    set_rgb(red,green,blue);
    
}

//LED Strip Driver Functions
LinuxDycoLEDStripDriver::LinuxDycoLEDStripDriver()
{
    _init = false;
    _strip_cnt = 0;
    _commcomp = false;
    clk_pin = false;
    
}

void LinuxDycoLEDStripDriver::init(uint16_t length)
{
    _led = new LinuxDycoLEDDriver[length];
    
    hal.gpio->pinMode(DYCODOUT,HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(DYCOCLK,HAL_GPIO_OUTPUT);
    hal.gpio->write(DYCOCLK,0);                     // both Clock pin and data pin at low state
    hal.gpio->write(DYCODOUT,0);
    
    _length = length;
    _init = true;
}

void LinuxDycoLEDStripDriver::set_solid_color(uint8_t led_num, uint8_t color)
{
    _led[led_num].set_solid_color(color);
}

//function for data transfer to dycoled and update with latest rgb values
bool LinuxDycoLEDStripDriver::update()
{
    bool data;
    uint16_t len = _length;
    uint32_t total_clk_pulses = (48+16*(len-1));
    if(!_init){
        return true;
    }
    
    if(clk_pulse_count < total_clk_pulses){
        hal.gpio->write(DYCOCLK, clk_pin = !clk_pin);       //toggle CLK pin
        
        if(clk_pin){
            if(clk_pulse_count < 32) {
                hal.gpio->write(DYCODOUT, 0);               //send 32 LOW bits to initiate data transfer
                //printf("0");
            } else if(clk_pulse_count >= 32 && clk_pulse_count % 16 == 0){
                hal.gpio->write(DYCODOUT, 1);               //set bit HIGH to enable led
                _led[_strip_cnt].reset();
                _strip_cnt++;
            } else if(clk_pulse_count > (uint16_t)(32+16*(_strip_cnt-1)) && clk_pulse_count < (uint16_t)(48+16*(_strip_cnt-1))){
                cntr = _strip_cnt - 1;
                data = _led[_strip_cnt-1].pop_data();       //pop rbg data bits out of the led data buffer
                hal.gpio->write(DYCODOUT, data);
            } 
            clk_pulse_count++;
        }
    } else{
        hal.gpio->write(DYCOCLK,0);         //transfer complete, reset CLK and DATA pins and
        hal.gpio->write(DYCODOUT,0);        //initialise flags and counters for next data transfer cycle
        _strip_cnt = 0;
        _commcomp = true;
        clk_pulse_count =0;
        for(int i=0; i<len; i++){
            _led[cntr].reset();             //prepare data buffer for next data transfer cycle
        }
        return true;
    }
    return false;
}

void LinuxDycoLEDStripDriver::set_pattern(uint16_t led_num,uint16_t color_series[],
                                          float bright_series[],uint16_t time_series[],
                                          uint8_t res, uint8_t step_cnt)
{
    _led[led_num].set_pattern(color_series,bright_series,time_series,res,step_cnt);
}

void LinuxDycoLEDStripDriver::generate_beat_pattern()
{
    if(_commcomp){
        for(int i=0;i<_length;i++){
            _led[i].pattern_step();
        }
        _commcomp = false;
    }
    update();
}


#endif