/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
    Analogin.cpp : We loop through the 16 pins and output the analog voltage of the pins
*/
#include <AP_HAL/AP_HAL.h>

void setup();    //declaration of the setup() function
void loop();     //declaration of the loop() function

const AP_HAL::HAL& hal = AP_HAL::get_HAL();    //create a reference to AP_HAL::HAL object to get access to hardware specific functions. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>  

AP_HAL::AnalogSource* chan;    //declare a pointer to AnalogSource object. AnalogSource class can be found in : AP_HAL->AnalogIn.h

// the setup function runs once when the board powers up
void setup(void) {
    hal.console->printf("Starting AP_HAL::AnalogIn test\r\n");    //print a starting message
    chan = hal.analogin->channel(0);    //initialization of chan variable. AnalogIn class can be found in : AP_HAL->AnalogIn.h
}

static int8_t pin;    //8 bit integer to hold the pin number.Pin number range is [0,15]

//the loop function runs over and over again forever
void loop(void) {
    //get the average voltage reading
    float v  = chan->voltage_average();    //note:the voltage value is divided into 1024 segments    
    //start a new line after going through the 16 pins
    if (pin == 0) {
        hal.console->printf("\n");
    }
    //print the voltage value(3 decimal places) alongside the pin number 
    hal.console->printf("[%u %.3f] ",
              (unsigned)pin, (double)v);
    //increment the pin number
    pin = (pin+1) % 16;
    //set pin corresponding to the new pin value
    IGNORE_RETURN(chan->set_pin(pin));
    //give a delay of 100ms
    hal.scheduler->delay(100);
}

AP_HAL_MAIN(); //HAL Macro that declares the main function. For more info see <https://ardupilot.org/dev/docs/learning-ardupilot-the-example-sketches.html/>
