
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AR_IR_Sensor_SITL_Analog.h"
#include "AR_IR_Sensor.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

extern const AP_HAL::HAL& hal;

AR_IR_SITL_Analog::AR_IR_SITL_Analog(AR_IR &val, AR_IR::AR_IR_State &val_state):
    AR_IR_Backend(val,val_state)
{
    _sitl = AP::sitl();
}

void AR_IR_SITL_Analog::read()
{
    const Location &home_loc = AP::ahrs().get_home();
    Location projected_home_line; 
    //location of sensors
    Location sensor_1; 
    Location sensor_2;

    float current_heading =  AP::ahrs().yaw_sensor * 0.01f;
    float max_sensor_range = 300.0f; // maximum range of the sensor (cm)
    projected_home_line.lat = home_loc.lat;
    projected_home_line.lng = home_loc.lng;
    projected_home_line.offset_bearing(0,1000); //1km line projected from home

    sensor_1.lat = _sitl->state.latitude*1.0e7;
    sensor_1.lng = _sitl->state.longitude*1.0e7;
    sensor_1.offset_bearing(wrap_360(current_heading+90),0.25f); //First sensor offset is set at 25cm

    sensor_2.lat = _sitl->state.latitude*1.0e7;
    sensor_2.lng = _sitl->state.longitude*1.0e7;
    sensor_2.offset_bearing(wrap_360(current_heading-90),0.25f); //Second sensor offset is set at -25cm

    //get vectors from origin to the points
    Vector2f start_NE, end_NE,sensor_1_offset_NE,sensor_2_offset_NE; 
    bool home_loc_vector = home_loc.get_vector_xy_from_origin_NE(start_NE);
    bool projected_line_vector = projected_home_line.get_vector_xy_from_origin_NE(end_NE);
    bool sensor_1_vector = sensor_1.get_vector_xy_from_origin_NE(sensor_1_offset_NE);
    bool sensor_2_vector = sensor_2.get_vector_xy_from_origin_NE(sensor_2_offset_NE);

    float crosstrack_error_1 = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, sensor_1_offset_NE); // distance between individual sensors and the virtual line
    float crosstrack_error_2 = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, sensor_2_offset_NE);
    
    //force sensor to stay in range limit
    if (crosstrack_error_1>= max_sensor_range) { 
        crosstrack_error_1 = max_sensor_range;
    } if (crosstrack_error_2>= max_sensor_range) {
        crosstrack_error_2 = max_sensor_range;
    }
    //convert the distance to voltage
    float voltage_sensor_1 = linear_interpolate(5.0f,0.0f,crosstrack_error_1,0.0f,max_sensor_range); 
    float voltage_sensor_2 = linear_interpolate(5.0f,0.0f,crosstrack_error_2,0.0f,max_sensor_range);

    //check for valid origin
    if (home_loc_vector&projected_line_vector&sensor_1_vector&sensor_2_vector) { 
        copy_state_to_frontend(voltage_sensor_1, voltage_sensor_2);
    } else {
         copy_state_to_frontend(0.0f, 0.0f); //set voltage 0 if valid origin is not available
    }
}

#endif