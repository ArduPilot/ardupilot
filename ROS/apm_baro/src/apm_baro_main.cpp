#include "ros/ros.h"
#include "apm_baro/apm_baro.h"
#include <AP_Baro.h>
#include <AP_HAL.h>
#include <AP_HAL_Linux.h>
#include <iostream>
#include <cstdlib>
#include <string>

//const HAL_Linux AP_HAL_Linux;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
static AP_Baro barometer;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apm_baro");

  std::string topic_name = std::string("apm_baro");
  //std::cout << topic_name << std::endl;

  ros::NodeHandle n;
  ros::Publisher baro_pub = n.advertise<apm_baro::apm_baro>(topic_name, 1000);
  apm_baro::apm_baro msg;

  // init the baro
  barometer.init();
  barometer.calibrate();  

  ros::Rate loop_rate(10);
  while (ros::ok()){
    barometer.accumulate();
    barometer.update();

    // Fetch the data from the baro abstraction
    msg.pressure = barometer.get_pressure();
    msg.temperature = barometer.get_temperature();
    msg.altitude = barometer.get_altitude();
    baro_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return 0;
}