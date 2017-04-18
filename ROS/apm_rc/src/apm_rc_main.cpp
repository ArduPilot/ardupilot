
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7


#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_HAL_Linux.h> 
#include <AP_HAL_Empty.h>
#include <AP_Baro.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include "../../libraries/AP_InertialSensor/AP_InertialSensor.h"
#include <AP_Notify.h>
#include <GCS_MAVLink.h>
#include <StorageManager.h>
#include <Filter.h>
#include "../../libraries/AP_Scheduler/AP_Scheduler.h"
#include "../../libraries/AP_HAL_Linux/UARTDriver.h"

#include "ros/ros.h"
#include "apm_rc/apm_rc.h"

#include <iostream>
#include <cstdlib>
#include <string>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define NUM_CHANNELS 8

static RC_Channel rc_1(CH_1);
static RC_Channel rc_2(CH_2);
static RC_Channel rc_3(CH_3);
static RC_Channel rc_4(CH_4);
static RC_Channel rc_5(CH_5);
static RC_Channel rc_6(CH_6);
static RC_Channel rc_7(CH_7);
static RC_Channel rc_8(CH_8);
static RC_Channel *rc = &rc_1;

# define ROLL_PITCH_INPUT_MAX      4500            // roll, pitch input range
# define THR_MAX 1000
# define THR_MIN 0

static void print_radio_values();
static void print_pwm(void);
//static void print_pwm_msg(void);

void setup()
{
    std::cout << "ArduPilot RC Channel test" << std::endl;
    print_radio_values();

    // set type of output
    rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
    rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
    rc_3.set_range(THR_MIN, THR_MAX);
    rc_4.set_angle(4500);

    rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    //set auxiliary servo ranges
    rc_5.set_range(0,1000);
    rc_6.set_range(0,1000);
    rc_7.set_range(0,1000);
    rc_8.set_range(0,1000);

//  INITIALIZE ROS
    char *argv[] = {"",NULL};
    int argc = sizeof(argv) / sizeof(char*) - 1;
    ros::init(argc,argv,"apm_rc");

//    std::string topic_name = std::string("apm_rc");
//    std::cout << topic_name << std::endl;

//    ros::NodeHandle n;
//    ros::Publisher rc_pub = n.advertise<apm_rc::apm_rc>(topic_name, 1000);
}

void loop()
{

   // while(ros::ok()){
    RC_Channel::set_pwm_all();

//Any way to do this just once??
    std::string topic_name = std::string("apm_rc");
    apm_rc::apm_rc msg;
    ros::NodeHandle n;
    ros::Publisher rc_pub = n.advertise<apm_rc::apm_rc>(topic_name, 1000);
//----------------------//
    for (int i=0; i<NUM_CHANNELS; i++) {
        msg.ch[i]=rc[i].control_in;
        std::cout << "ch" << (unsigned)i+1<<" " <<(int) msg.ch[i] << " ";
     }
     std::cout << std::endl;

     rc_pub.publish(msg);

     ros::spinOnce();
     ros::Rate loop_rate(10);
     loop_rate.sleep();
}


static void print_radio_values()
{
    for (int i=0; i<NUM_CHANNELS; i++) {
        std::cout << "CH" << (unsigned)i+1 << ": "<< (unsigned)rc[i].radio_min << "|" << (unsigned)rc[i].radio_max<< std::endl;
    }
}

AP_HAL_MAIN();
