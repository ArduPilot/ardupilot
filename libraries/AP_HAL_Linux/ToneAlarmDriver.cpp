#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "ToneAlarmDriver.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

using namespace Linux;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
#include "GPIO.h"
#define RASPILOT_TONE_PIN RPI_GPIO_18
#endif

extern const AP_HAL::HAL& hal;
static uint16_t notes[] = { 0,
NOTE_C4, NOTE_CS4, NOTE_D4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_AS4, NOTE_B4,
NOTE_C5, NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_E5, NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_GS5, NOTE_A5, NOTE_AS5, NOTE_B5,
NOTE_C6, NOTE_CS6, NOTE_D6, NOTE_DS6, NOTE_E6, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_GS6, NOTE_A6, NOTE_AS6, NOTE_B6,
NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_E7, NOTE_F7, NOTE_FS7, NOTE_G7, NOTE_GS7, NOTE_A7, NOTE_AS7, NOTE_B7
};

//List of RTTTL tones
const char* ToneAlarm::tune[TONE_NUMBER_OF_TUNES] = {
                                "Startup:d=8,o=6,b=480:a,d7,c7,a,d7,c7,a,d7,16d7,16c7,16d7,16c7,16d7,16c7,16d7,16c7",
                                "Error:d=4,o=6,b=400:8a,8a,8a,p,a,a,a,p",
                                "notify_pos:d=4,o=6,b=400:8e,8e,a",
                                "notify_neut:d=4,o=6,b=400:8e,e",
                                "notify_neg:d=4,o=6,b=400:8e,8c,8e,8c,8e,8c",
                                "arming_warn:d=1,o=4,b=75:g",
                                "batt_war_slow:d=4,o=6,b=200:8a",
                                "batt_war_fast:d=4,o=6,b=512:8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a",
                                "GPS_war:d=4,o=6,b=512:a,a,a,1f#",
                                "Arm_fail:d=4,o=4,b=512:b,a,p",
                                "para_rel:d=16,o=6,b=512:a,g,a,g,a,g,a,g"};
//Tune Repeat true: play rtttl tune in loop, false: play only once
bool ToneAlarm::tune_repeat[TONE_NUMBER_OF_TUNES] = {false,true,false,false,false,false,true,true,false,false,false};

ToneAlarm::ToneAlarm()
{
    period_fd = open("/sys/devices/ocp.3/pwm_test_P8_36.12/period",O_WRONLY);
    duty_fd = open("/sys/devices/ocp.3/pwm_test_P8_36.12/duty",O_WRONLY);
    run_fd = open("/sys/devices/ocp.3/pwm_test_P8_36.12/run",O_WRONLY);

    tune_num = -1;                    //initialy no tune to play
    tune_pos = 0;
}

bool ToneAlarm::init()
{
    tune_num = 0;                    //play startup tune

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
    hal.gpio->pinMode(RASPILOT_TONE_PIN, HAL_GPIO_ALT, 5);
#else
    if((period_fd == -1) || (duty_fd == -1) || (run_fd == -1)){
        hal.console->printf("ToneAlarm: Error!! please check if PWM overlays are loaded correctly");
        return false;
    }
#endif

    return true;
}

void ToneAlarm::set_tune(uint8_t tone)
{
    tune_num = tone;
}

bool ToneAlarm::is_tune_comp()
{
    return tune_comp;
}

void ToneAlarm::stop()
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
    hal.gpio->setPWMDuty(RASPILOT_TONE_PIN, 0);
#else
    dprintf(run_fd,"0");
#endif
}

bool ToneAlarm::play()
{
    uint16_t cur_time = hal.scheduler->millis();
    if(tune_num != prev_tune_num){
        tune_changed = true;
        return true;
    }
    if(cur_note != 0){
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
        hal.gpio->setPWMPeriod(RASPILOT_TONE_PIN, 1000000/cur_note);
        hal.gpio->setPWMDuty(RASPILOT_TONE_PIN, 50);
#else
        dprintf(run_fd,"0");
        dprintf(period_fd,"%u",1000000000/cur_note);
        dprintf(duty_fd,"%u",500000000/cur_note);
        dprintf(run_fd,"1");
#endif
        cur_note =0;
        prev_time = cur_time;
    }
    if((cur_time - prev_time) > duration){
        stop();
        if(tune[tune_num][tune_pos] == '\0'){
            if(!tune_repeat[tune_num]){
                tune_num = -1;
            }

            tune_pos = 0;
            tune_comp = true;
            return false;
        }
        return true;
    }
    return false;
}

bool ToneAlarm::set_note(){
    // first, get note duration, if available
    uint16_t scale,note,num =0;
    duration = 0;

    while(isdigit(tune[tune_num][tune_pos])){                   //this is a safe while loop as it can't go further than 
                                                                //the length of the rtttl tone string
        num = (num * 10) + (tune[tune_num][tune_pos++] - '0');
    }
    if(num){
        duration = wholenote / num;
    } else{
        duration = wholenote / 4;  // we will need to check if we are a dotted note after
    }
    // now get the note
    note = 0;

    switch(tune[tune_num][tune_pos]){
        case 'c':
        note = 1;
        break;
        case 'd':
        note = 3;
        break;
        case 'e':
        note = 5;
        break;
        case 'f':
        note = 6;
        break;
        case 'g':
        note = 8;
        break;
        case 'a':
        note = 10;
        break;
        case 'b':
        note = 12;
        break;
        case 'p':
        default:
        note = 0;
    }

    tune_pos++;

    // now, get optional '#' sharp
    if(tune[tune_num][tune_pos] == '#'){
        note++;
        tune_pos++;
    }

    // now, get optional '.' dotted note

    if(tune[tune_num][tune_pos] == '.'){
        duration += duration/2;
        tune_pos++;
    }

    // now, get scale

    if(isdigit(tune[tune_num][tune_pos])){
        scale = tune[tune_num][tune_pos] - '0';
        tune_pos++;
    } else{
        scale = default_oct;
    }

    scale += OCTAVE_OFFSET;

    if(tune[tune_num][tune_pos] == ','){
        tune_pos++;       // skip comma for next note (or we may be at the end)
    }
    // now play the note

    if(note){
        if(tune_changed == true){
            tune_pos =0;
            tune_changed = false;
        }
        cur_note = notes[(scale - 4) * 12 + note];
        return true;
    } else{
        cur_note = 0;
        return true;
    }
}

bool ToneAlarm::init_tune(){

    uint16_t num;
    default_dur = 4;
    default_oct = 6;
    bpm = 63;
    prev_tune_num = tune_num;
    if(tune_num <0 || tune_num > TONE_NUMBER_OF_TUNES){
        return false;
    }

    tune_comp = false;
    while(tune[tune_num][tune_pos] != ':'){
        if(tune[tune_num][tune_pos] == '\0'){
            return false;
        }
        tune_pos++;
    }
    tune_pos++;

    if(tune[tune_num][tune_pos] == 'd'){
        tune_pos+=2;
        num = 0;

        while(isdigit(tune[tune_num][tune_pos])){
            num = (num * 10) + (tune[tune_num][tune_pos++] - '0');
        }
        if(num > 0){
            default_dur = num;
        }
        tune_pos++;                   // skip comma
    }


    // get default octave

    if(tune[tune_num][tune_pos] == 'o')
    {
        tune_pos+=2;              // skip "o="
        num = tune[tune_num][tune_pos++] - '0';
        if(num >= 3 && num <=7){
            default_oct = num;
        }
        tune_pos++;                   // skip comma
    }

    // get BPM

    if(tune[tune_num][tune_pos] == 'b'){
        tune_pos+=2;              // skip "b="
        num = 0;
        while(isdigit(tune[tune_num][tune_pos])){
            num = (num * 10) + (tune[tune_num][tune_pos++] - '0');
        }
        bpm = num;
        tune_pos++;                   // skip colon
    }

    // BPM usually expresses the number of quarter notes per minute
    wholenote = (60 * 1000L / bpm) * 4;  // this is the time for whole note (in milliseconds)
    return true;
}


#endif
