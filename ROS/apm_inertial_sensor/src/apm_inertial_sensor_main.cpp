#include "ros/ros.h"
#include "apm_inertial_sensor/apm_imu.h"
#include <AP_InertialSensor.h>
#include <AP_HAL.h>
#include <AP_HAL_Linux.h>
#include <iostream>
#include <cstdlib>
#include <string>

//const HAL_Linux AP_HAL_Linux;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_InertialSensor ins;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apm_inertial_sensor");

  std::string topic_name = std::string("apm_inertial_sensor");
  //std::cout << topic_name << std::endl;

  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<apm_inertial_sensor::apm_imu>(topic_name, 1000);
  apm_inertial_sensor::apm_imu msg;

  // init the IMU
    ins.init(AP_InertialSensor::COLD_START, 
             AP_InertialSensor::RATE_100HZ);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    ins.wait_for_sample();
    // Update IMU values
    ins.update();
    // gyro = ins.get_gyro();
    // accel = ins.get_accel();

    // Fetch the data from the imu abstraction
    msg.gyro = {ins.get_gyro().x, ins.get_gyro().y, ins.get_gyro().z};
    msg.accel = {ins.get_accel().x, ins.get_accel().y, ins.get_accel().z};

    imu_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return 0;
}