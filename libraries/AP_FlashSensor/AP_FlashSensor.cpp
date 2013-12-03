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

// return true if the Voltage is greater than threshold voltage (set in ArduPlane/APM_Config.h)
bool AP_FlashSensor::test_voltage()
{
    if (read_average() > FLASH_SENSOR_TRESHOLD)
    {
        // another photo has been taken
        if(!is_hot)
        {
            frame_counter ++;
            // get current telemetry and que MavLink message
            send_telemetry();
        }
        is_hot = true;
    }
    else
    {
        is_hot = false;
    }
}

void AP_FlashSensor::reset_counter()
{
    frame_counter = 0;
}
