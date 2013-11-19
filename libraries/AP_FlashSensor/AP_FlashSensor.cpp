/* 
KSU AIAA Code:
    Class designed to use a camera flash signal on an analog 
    input in order to send GCS telemetry for each still image.

    Date Update: 11/13/2013
    Author(s): Joel Cranmer [Matlock42],
*/

#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_ADC.h>
#include <AP_FlashSensor.h>

void AP_FlashSensor::send_telemetry()
{
    // create a new message for queue with the current 
    // telemetry data and the image number
}

// return the result of Voltage greater than threshold
bool AP_FlashSensor::test_voltage()
{
    if (read_average() > flash_threshold_voltage)
    {
        frame_counter ++;
        return true;
    }
    else
    {
        return false;
    }
}

void AP_FlashSensor::reset_counter()
{
    frame_counter = 0;
}