/*
 *       Example sketch to demonstrate use of LowPassFilter library.
 *       Code by Randy Mackay. DIYDrones.com
 */

#include <Filter/Filter.h>                     // Filter library
#include <AP_Eigen/AP_Eigen.h>
#include <AP_HAL/AP_HAL.h>


const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// create a global instance of the class
LowPassFilter<Vector3f> low_pass_filter_AP;
LowPassFilter<AP_Eigen::Vector3f> low_pass_filter_Eigen;

// setup routine
void setup()
{
    // introduction
    hal.console->printf("ArduPilot LowPassFilter test ver 1.0\n\n");

    // set-up filter
    low_pass_filter_AP.set_cutoff_frequency(1.0f);
    low_pass_filter_Eigen.set_cutoff_frequency(1.0f);

    // Wait for the serial connection
    hal.scheduler->delay(500);
}

//Main loop where the action takes place
void loop()
{
    int i;
    float new_value;

    // reset value to 100.  If not reset the filter will start at the first value entered
    low_pass_filter_AP.reset(Vector3f(0,0,0));
    low_pass_filter_Eigen.reset(AP_Eigen::Vector3f(0,0,0));

    int timer = hal.scheduler->millis();
    Vector3f filtered_value_AP;
    Vector3f new_vec_AP;
    for( i=0; i<5000000; i++ ) {

        // new data value
        new_value = sinf((float)i*2*PI*5/50.0f);  // 5hz
        new_vec_AP = Vector3f(new_value,new_value,new_value);

        // apply new value and retrieved filtered result
        filtered_value_AP = low_pass_filter_AP.apply(new_vec_AP, 0.02f);
    }
    hal.console->printf("Time for AP types: %d\n",  hal.scheduler->millis() - timer);
    hal.scheduler->delay(1000);
  
    timer = hal.scheduler->millis();
    AP_Eigen::Vector3f filtered_value_Eigen;
    AP_Eigen::Vector3f new_vec_Eigen;
    for( i=0; i<5000000; i++ ) {
        // new data value
        new_value = sinf((float)i*2*PI*5/50.0f);  // 5hz
        new_vec_Eigen = AP_Eigen::Vector3f(new_value,new_value,new_value);

        // apply new value and retrieved filtered result
        filtered_value_Eigen = low_pass_filter_Eigen.apply(new_vec_Eigen, 0.02f);
    }
    hal.console->printf("Time for Eigen types: %d\n",  hal.scheduler->millis() - timer);
}

AP_HAL_MAIN();
