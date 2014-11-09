#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "Util.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

using namespace Linux;

extern const AP_HAL::HAL& hal;
static uint16_t notes[] = { 0,
NOTE_C4, NOTE_CS4, NOTE_D4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_AS4, NOTE_B4,
NOTE_C5, NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_E5, NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_GS5, NOTE_A5, NOTE_AS5, NOTE_B5,
NOTE_C6, NOTE_CS6, NOTE_D6, NOTE_DS6, NOTE_E6, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_GS6, NOTE_A6, NOTE_AS6, NOTE_B6,
NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_E7, NOTE_F7, NOTE_FS7, NOTE_G7, NOTE_GS7, NOTE_A7, NOTE_AS7, NOTE_B7
};

void LinuxUtil::toneAlarm()
{
    tune[TONE_STARTUP_TUNE] = "Startup:d=8,o=6,b=240:a,d7,c7,a,d7,c7,a,d7,16d7,16c7,16d7,16c7,16d7,16c7,16d7,16c7";
    tune[TONE_ERROR_TUNE] = "Error:d=4,o=6,b=200:8a,8a,8a,p,a,a,a,p";
    tune[TONE_NOTIFY_POSITIVE_TUNE] = "notify_pos:d=4,o=6,b=200:8e,8e,a";
    tune[TONE_NOTIFY_NEUTRAL_TUNE] = "notify_neut:d=4,o=6,b=200:8e,e";
    tune[TONE_NOTIFY_NEGATIVE_TUNE] = "notify_neg:d=4,o=6,b=200:8e,8c,8e,8c,8e,8c";
    tune[TONE_ARMING_WARNING_TUNE] = "arming_warn:d=1,o=4,b=75:g";
    tune[TONE_BATTERY_WARNING_SLOW_TUNE] = "batt_war_slow:d=4,o=6,b=100:8a";
    tune[TONE_BATTERY_WARNING_FAST_TUNE] = "batt_war_fast:d=4,o=6,b=255:8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a";
    tune[TONE_GPS_WARNING_TUNE] = "GPS_war:d=4,o=6,b=255:a,a,a,1f#";
    tune[TONE_ARMING_FAILURE_TUNE] = "Arm_fail:d=4,o=4,b=255:b,a,p";
    tune[TONE_PARACHUTE_RELEASE_TUNE] = "para_rel:d=16,o=6,b=255:a,g,a,g,a,g,a,g";
    
    tune_repeat[TONE_STARTUP_TUNE] = false;
    tune_repeat[TONE_ERROR_TUNE] = true;
    tune_repeat[TONE_NOTIFY_POSITIVE_TUNE] = false; 
    tune_repeat[TONE_NOTIFY_NEUTRAL_TUNE] = false;
    tune_repeat[TONE_NOTIFY_NEGATIVE_TUNE] = false;
    tune_repeat[TONE_ARMING_WARNING_TUNE] = false;
    tune_repeat[TONE_BATTERY_WARNING_SLOW_TUNE] = true;
    tune_repeat[TONE_BATTERY_WARNING_FAST_TUNE] = true;
    tune_repeat[TONE_GPS_WARNING_TUNE] = false;
    tune_repeat[TONE_ARMING_FAILURE_TUNE] = false;
    tune_repeat[TONE_PARACHUTE_RELEASE_TUNE] = false;
    
    char str[]="/sys/devices/ocp.3/pwm_test_P8_36.12/period";
    char str1[]="/sys/devices/ocp.3/pwm_test_P8_36.12/duty";
    char str2[]="/sys/devices/ocp.3/pwm_test_P8_36.12/run";
    
    period_fd = open(str,O_WRONLY);
    duty_fd = open(str1,O_WRONLY);
    run_fd = open(str2,O_WRONLY);
    
    tune_num = -1;                    //initialy no tune to play
}
int8_t LinuxUtil::toneAlarm_init()
{
    tune_num = 0;                    //play startup tune
    if((period_fd == -1) || (duty_fd == -1) || (run_fd == -1)){
        return -1;
    }
    return 0;
}

void LinuxUtil::stop()
{

    write(run_fd,"0",sizeof(char));
}

void LinuxUtil::play(int tone,int duration)
{
    if(tune_num != prev_tune_num){
        tune_changed = true;
        return;
    }
    if(tone != 0){
        dprintf(run_fd,"0");
        dprintf(period_fd,"%u",1000000000/tone);    
        dprintf(duty_fd,"%u",500000000/tone);
        dprintf(run_fd,"1");
    }
    hal.scheduler->delay(duration);
}

void LinuxUtil::toneAlarm_set_tune(uint8_t tone)
{
    tune_num = tone;
}

void LinuxUtil::play_tune()
{
    if(tune_num < 0 || tune_num > 10){
        return;
    }
    
    uint32_t p = 0;
    uint8_t default_dur = 4;
    uint8_t default_oct = 6;
    uint16_t bpm = 63;
    uint16_t num;
    uint32_t wholenote;
    uint32_t duration;
    uint8_t note;
    uint8_t scale;
    prev_tune_num = tune_num;
    while(1){
        while(tune[tune_num][p] != ':'){
            p++;
        }
        p++;                     

        if(tune[tune_num][p] == 'd'){
            p+=2;              
            num = 0;

            while(isdigit(tune[tune_num][p])){
                num = (num * 10) + (tune[tune_num][p++] - '0');
            }
            if(num > 0){ 
                default_dur = num;
            }        
            p++;                   // skip comma
        }


        // get default octave

        if(tune[tune_num][p] == 'o')
        {
            p+=2;              // skip "o="
            num = tune[tune_num][p++] - '0';
            if(num >= 3 && num <=7){
                default_oct = num;
            }
            p++;                   // skip comma
        }

        // get BPM

        if(tune[tune_num][p] == 'b'){
            p+=2;              // skip "b="
            num = 0;
            while(isdigit(tune[tune_num][p])){
                num = (num * 10) + (tune[tune_num][p++] - '0');
            }
            bpm = num;
            p++;                   // skip colon
        }

        // BPM usually expresses the number of quarter notes per minute
        wholenote = (60 * 1000L / bpm) * 4;  // this is the time for whole note (in milliseconds)


        // now begin note loop
        while(tune[tune_num][p]){
            // first, get note duration, if available
            num = 0;
            while(isdigit(tune[tune_num][p])){
                num = (num * 10) + (tune[tune_num][p++] - '0');
            }
            if(num){
                duration = wholenote / num;
            } else{
                duration = wholenote / default_dur;  // we will need to check if we are a dotted note after
            }        
            // now get the note
            note = 0;

            switch(tune[tune_num][p]){
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

            p++;

            // now, get optional '#' sharp
            if(tune[tune_num][p] == '#'){
                note++;
                p++;
            }

            // now, get optional '.' dotted note

            if(tune[tune_num][p] == '.'){
                duration += duration/2;
                p++;
            }

            // now, get scale

            if(isdigit(tune[tune_num][p])){
                scale = tune[tune_num][p] - '0';
                p++;
            } else{
                scale = default_oct;
            }

            scale += OCTAVE_OFFSET;

            if(tune[tune_num][p] == ','){
                p++;       // skip comma for next note (or we may be at the end)
            }

            // now play the note

            if(note){
                play(notes[(scale - 4) * 12 + note],duration);
                if(tune_changed == true){
                    tune_changed = false;
                    return;
                }
                stop();
            } else{
                hal.scheduler->delay(duration);
            }
        }
        
        if(tune_repeat[tune_num]){
            continue;
        } else{
            tune_num = -1;
            return;
        }
    }
}

void LinuxUtil::_toneAlarm_timer_tick(){
    play_tune();
}
#endif
