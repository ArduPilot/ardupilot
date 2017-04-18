#include "ros/ros.h"
#include "apm_gps/apm_gps.h"
#include <AP_GPS.h>
#include <AP_HAL.h>
#include <AP_HAL_Linux.h>
#include <iostream>
#include <cstdlib>
#include <string>

//const HAL_Linux AP_HAL_Linux;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
static AP_GPS gps;
static AP_SerialManager serial_manager;
static DataFlash_File DataFlash(HAL_BOARD_LOG_DIRECTORY);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apm_gps");

  std::string topic_name = std::string("apm_gps");
  //std::cout << topic_name << std::endl;

  ros::NodeHandle n;
  ros::Publisher gps_pub = n.advertise<apm_gps::apm_gps>(topic_name, 1000);
  apm_gps::apm_gps msg;

  // init the gps
  gps.init(&DataFlash, serial_manager);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    gps.update();

    // Fetch the data from the gps abstraction
    msg.status = gps.status();
    msg.time_week_ms = gps.time_week_ms();
    msg.time_week = gps.time_week();
    msg.alt = gps.location().alt;
    msg.lat = gps.location().lat;
    msg.lng = gps.location().lng;
    msg.ground_speed = gps.ground_speed();
    msg.hdop = gps.get_hdop();
    msg.num_sats = gps.num_sats();
    msg.velocity_x = gps.velocity().x;
    msg.velocity_y = gps.velocity().y;
    msg.velocity_z = gps.velocity().z;

    gps_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return 0;
}