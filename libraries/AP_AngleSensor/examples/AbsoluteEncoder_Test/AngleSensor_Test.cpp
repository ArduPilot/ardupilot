/*
 *  RangeFinder test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_AngleSensor/AP_AngleSensor_Backend.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Math/definitions.h>    

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_ANGLESENSOR_ENABLED
    static AP_AngleSensor angle_sensor_driver;
#endif

void setup()
{
    #if AP_ANGLESENSOR_ENABLED
        // print welcome message
        hal.console->printf("Angle Sensor library test\n");

        // setup for analog pin 13
        AP_Param::set_object_value(&angle_sensor_driver, angle_sensor_driver.var_info, "_TYPE", (uint8_t)AP_AngleSensor::AngleSensor_Type::ANGLESENSOR_TYPE_AS5048B);
        AP_Param::set_object_value(&angle_sensor_driver, angle_sensor_driver.var_info, "_BUS", (uint8_t)0);
        AP_Param::set_object_value(&angle_sensor_driver, angle_sensor_driver.var_info, "_OFFS", (float)-354.57);
        AP_Param::set_object_value(&angle_sensor_driver, angle_sensor_driver.var_info, "_DIR", (int8_t)-1);

        // initialise sensor, delaying to make debug easier
        hal.scheduler->delay(2000);
        angle_sensor_driver.init();
        hal.console->printf("Angle Sensor: %d device(s) initialized\n", angle_sensor_driver.num_sensors());
    #endif
}

void loop()
{
    #if AP_ANGLESENSOR_ENABLED
        // Delay between reads
        hal.scheduler->delay(100);
        angle_sensor_driver.update();

        bool had_data = false;
        for (uint8_t i=0; i<angle_sensor_driver.num_sensors(); i++) {
            if (!angle_sensor_driver.healthy(i)) {
                hal.console->printf("device_%u type %d not healthy\n",
                                    i,
                                    (int)angle_sensor_driver.get_type(i));
                continue;
            }
            hal.console->printf("device_%u type %d angle %f quality %d\n",
                                i,
                                (int)angle_sensor_driver.get_type(i),
                                (float)angle_sensor_driver.get_angle_radians(i)*180/M_PI,
                                (int)angle_sensor_driver.get_signal_quality(i));
            had_data = true;
        }
        if (!had_data) {
            hal.console->printf("All: no data on any sensor\n");
        }
    #else
        hal.console->printf("AngleSensor not available\n");
        hal.scheduler->delay(1000);
    #endif

}
AP_HAL_MAIN();
