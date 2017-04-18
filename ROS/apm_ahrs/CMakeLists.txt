cmake_minimum_required(VERSION 2.8.3)
project(apm_ahrs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES apm_ahrs.msg)

 # Hardware Abstraction Layer APM ROS definitions
 #		TODO: create another package that abstracts
 #				all boards
add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_LINUX)
add_definitions(-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_LINUX_PXF)
add_definitions(-DSKETCHNAME="APMRover2")

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}	
	../../libraries/AP_InertialSensor
	../../libraries/AP_Progmem
	../../libraries/AP_HAL
	../../libraries/AP_HAL/utility
	../../libraries/AP_HAL_Linux
	../../libraries/AP_HAL_Empty
	../../libraries/AP_Math
	../../libraries/AP_Common
	../../libraries/AP_Param
	../../libraries/Filter
	../../libraries/GCS_MAVLink
	../../libraries/AP_Notify
	../../libraries/AP_Vehicle
	../../libraries/AP_HAL_Linux
	../../libraries/AP_GPS
	../../libraries/StorageManager
	../../libraries/Filter
	../../libraries/AP_ADC
	../../libraries/AP_AHRS
	../../libraries/AP_Compass
	../../libraries/AP_Declination
	../../libraries/AP_Airspeed
	../../libraries/AP_Baro
	../../libraries/AP_NavEKF
	../../libraries/AP_SerialManager
	../../libraries/AP_ADC_AnalogSource
	../../libraries/Dataflash
	../../libraries/GCS_MAVLink
	../../libraries/GCS_MAVLink/include/mavlink/v1.0
	../../libraries/GCS_MAVLink/include/mavlink/v1.0/common
	../../libraries/GCS_MAVLink/include/mavlink/v1.0/ardupilotmega
	../../libraries/GCS_MAVLink/include/mavlink/v0.9
	../../libraries/GCS_MAVLink/include/mavlink/v0.9/common
	../../libraries/GCS_MAVLink/include/mavlink/v0.9/ardupilotmega)

#SET(CMAKE_CXX_FLAGS "fpermissive")
add_definitions ( -fpermissive)

add_executable(apm_ahrs_main src/apm_ahrs_main.cpp	
	../../libraries/AP_HAL/utility/ftoa_engine.cpp
	../../libraries/AP_HAL/utility/Print.cpp
	../../libraries/AP_HAL/utility/print_vprintf.cpp
	../../libraries/AP_HAL/utility/utoa_invert.cpp
	../../libraries/AP_HAL/UARTDriver.cpp
	../../libraries/AP_HAL_Linux/HAL_Linux_Class.cpp
	../../libraries/AP_HAL_Linux/AnalogIn.cpp
	../../libraries/AP_HAL_Linux/dsm.cpp
	../../libraries/AP_HAL_Linux/GPIO.cpp
	../../libraries/AP_HAL_Linux/GPIO_BBB.cpp
	../../libraries/AP_HAL_Linux/I2CDriver.cpp
	../../libraries/AP_HAL_Linux/RCInput.cpp
	../../libraries/AP_HAL_Linux/RCInput_PRU.cpp
	../../libraries/AP_HAL_Linux/RCOutput_PRU.cpp
	../../libraries/AP_HAL_Linux/sbus.cpp
	../../libraries/AP_HAL_Linux/Scheduler.cpp
	../../libraries/AP_HAL_Linux/Semaphores.cpp
	../../libraries/AP_HAL_Linux/SPIDriver.cpp
	../../libraries/AP_HAL_Linux/Storage.cpp
	../../libraries/AP_HAL_Linux/Storage_FRAM.cpp
	../../libraries/AP_HAL_Linux/ToneAlarmDriver.cpp
	../../libraries/AP_HAL_Linux/UARTDriver.cpp
	../../libraries/AP_HAL_Linux/Util.cpp
	../../libraries/AP_InertialSensor/AP_InertialSensor.cpp
	../../libraries/AP_InertialSensor/AP_InertialSensor_MPU9250.cpp
	../../libraries/AP_InertialSensor/AP_InertialSensor_HIL.cpp
	../../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
	../../libraries/AP_Notify/AP_Notify.cpp
	../../libraries/AP_Notify/RGBLed.cpp
	../../libraries/AP_Notify/ToshibaLED.cpp
	../../libraries/AP_Notify/ToshibaLED_I2C.cpp
	../../libraries/AP_Notify/AP_BoardLED.cpp
	../../libraries/AP_Notify/ToneAlarm_Linux.cpp
	../../libraries/AP_Param/AP_Param.cpp
	../../libraries/AP_Math/AP_Math.cpp
	../../libraries/AP_Math/vector3.cpp
	../../libraries/StorageManager/StorageManager.cpp
	../../libraries/Filter/LowPassFilter2p.cpp
	../../libraries/AP_AHRS/AP_AHRS.cpp
	../../libraries/AP_AHRS/AP_AHRS_DCM.cpp
	../../libraries/AP_AHRS/AP_AHRS_NavEKF.cpp
	../../libraries/AP_Compass/Compass.cpp
	../../libraries/AP_Compass/AP_Compass_AK8963.cpp
	../../libraries/AP_Compass/AP_Compass_Backend.cpp
	../../libraries/AP_Compass/AP_Compass_HIL.cpp
	../../libraries/AP_Compass/AP_Compass_HMC5843.cpp
	../../libraries/AP_Declination/AP_Declination.cpp
	../../libraries/AP_Airspeed/AP_Airspeed.cpp
	../../libraries/AP_Baro/AP_Baro.cpp
	../../libraries/AP_Baro/AP_Baro_MS5611.cpp
	../../libraries/AP_Baro/AP_Baro_Backend.cpp
	../../libraries/AP_Baro/AP_Baro_HIL.cpp
	../../libraries/AP_SerialManager/AP_SerialManager.cpp
	../../libraries/AP_GPS/AP_GPS.cpp
	../../libraries/AP_GPS/AP_GPS_UBLOX.cpp
	../../libraries/AP_GPS/AP_GPS_SIRF.cpp
	../../libraries/AP_GPS/AP_GPS_MTK.cpp	
	../../libraries/AP_GPS/AP_GPS_SBP.cpp		
	../../libraries/AP_GPS/GPS_Backend.cpp
	../../libraries/AP_GPS/AP_GPS_NMEA.cpp
	../../libraries/AP_GPS/AP_GPS_MTK19.cpp
	../../libraries/AP_Math/vector2.cpp
	../../libraries/AP_Math/matrix3.cpp
	../../libraries/AP_Math/location.cpp
	../../libraries/AP_Math/quaternion.cpp
	../../libraries/AP_Airspeed/AP_Airspeed_analog.cpp
	../../libraries/AP_Airspeed/AP_Airspeed_I2C.cpp
	../../libraries/AP_Airspeed/Airspeed_Calibration.cpp
	../../libraries/AP_NavEKF/AP_NavEKF.cpp
	../../libraries/Filter/DerivativeFilter.cpp
	../../libraries/AP_Math/edc.cpp)

target_link_libraries(apm_ahrs_main ${catkin_LIBRARIES})
add_dependencies(apm_ahrs_main apm_ahrs_generate_messages_cpp)

file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/examples/*)
  catkin_install_python(PROGRAMS
  ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	TARGETS apm_ahrs_main
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})