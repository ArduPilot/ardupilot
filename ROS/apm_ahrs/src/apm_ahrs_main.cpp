#include "ros/ros.h"
#include "apm_inertial_sensor/apm_imu.h"
#include "apm_baro/apm_baro.h"
#include "apm_ahrs/apm_ahrs.h"
#include <AP_InertialSensor.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <AP_SerialManager.h>
#include <AP_HAL.h>
#include <AP_HAL_Linux.h>
#include <iostream>
#include <cstdlib>
#include <string>

//const HAL_Linux AP_HAL_Linux;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_InertialSensor ins;
Compass compass;
AP_GPS gps;
AP_Baro baro;
AP_SerialManager serial_manager;

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(ins, baro, gps);

void imu_callback(const apm_inertial_sensor::apm_imu::ConstPtr& msg){
  // ROS_INFO("I heard:\n gyro: %f %f %f\n accel: %f %f %f\n", msg->gyro.x, 
  //    msg->gyro.y, msg->gyro.z, msg->accel.x, msg->accel.y, msg->accel.z);
  
  // for now assume there's only one instance
  ins.set_gyro(0, Vector3f(msg->gyro[0], msg->gyro[1], msg->gyro[2]));
  ins.set_accel(0, Vector3f(msg->accel[0], msg->accel[1], msg->accel[2]));  
}

void baro_callback(const apm_baro::apm_baro::ConstPtr& msg){
  // ROS_INFO("I heard:\n baro: temperature: %f pressure: %f altitude: %f\n", 
  //    msg->baro.temperature, msg->baro.pressure, msg->baro.altitude);
  
  // for now assume there's only one instance
  baro.set_temperature(0, msg->temperature);
  baro.set_pressure(0, msg->pressure);
  baro.set_altitude(0, msg->altitude);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "apm_ahrs");
  std::string topic_name = std::string("apm_ahrs");
  //std::cout << topic_name << std::endl;
  ros::NodeHandle n;

  // Susbscribe
  ros::Subscriber sub_imu, sub_baro;  // sub_gps;  
  sub_imu = n.subscribe("apm_inertial_sensor", 1000, imu_callback);
  sub_baro = n.subscribe("apm_baro", 1000, baro_callback);
  // sub_gps = n.subscribe("apm_gps", 1000, imu_callback);

  // Publish
  ros::Publisher ahrs_pub = n.advertise<apm_ahrs::apm_ahrs>(topic_name, 1000);
  apm_ahrs::apm_ahrs msg;

  ins.init(AP_InertialSensor::COLD_START, 
       AP_InertialSensor::RATE_100HZ);
  ahrs.init();
  serial_manager.init();

  if( compass.init() ) {
      ROS_INFO("Enabling compass\n"); 
      ahrs.set_compass(&compass);
  } else {
      ROS_INFO("No compass detected\n");
  }
  gps.init(NULL, serial_manager);


  ros::Rate loop_rate(10);
  while (ros::ok()){

    // Do the fetching
    static uint16_t counter;
    float heading = 0;

    if (compass.read()) {
      heading = compass.calculate_heading(ahrs.get_dcm_matrix());
#if WITH_GPS
      g_gps->update();
#endif
    }

    ahrs.update();
    counter++;

    Vector3f drift  = ahrs.get_gyro_drift();
    ROS_INFO("r:%4.1f  p:%4.1f y:%4.1f "
                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f\n",
                        ToDeg(ahrs.roll),
                        ToDeg(ahrs.pitch),
                        ToDeg(ahrs.yaw),
                        ToDeg(drift.x),
                        ToDeg(drift.y),
                        ToDeg(drift.z),
                        compass.use_for_yaw() ? ToDeg(heading) : 0.0 );

    // Fetch the data from the abstraction and publish it
    msg.pitch = ahrs.pitch;
    msg.roll = ahrs.roll;
    msg.yaw = ahrs.yaw;    
    ahrs_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }  
  return 0;
}