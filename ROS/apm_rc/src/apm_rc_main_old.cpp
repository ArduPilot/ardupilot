#include "ros/ros.h"
//#include "apm_rc/apm_rc.h"
#include "../../../libraries/RC_Channel/RC_Channel.h"
#include <AP_HAL.h>
#include <AP_HAL_Linux.h>
#include <iostream>
#include <cstdlib>
#include <string>

//const HAL_Linux AP_HAL_Linux;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define NUM_CHANNELS 8

# define ROLL_PITCH_INPUT_MAX      4500            // roll, pitch input range
# define THR_MAX 1000
# define THR_MIN 0


static RC_Channel rc_1(CH_1);
static RC_Channel rc_2(CH_2);
static RC_Channel rc_3(CH_3);
static RC_Channel rc_4(CH_4);
static RC_Channel rc_5(CH_5);
static RC_Channel rc_6(CH_6);
static RC_Channel rc_7(CH_7);
static RC_Channel rc_8(CH_8);
static RC_Channel *rc = &rc_1;


void setup(){
	std::cout <<"RC channel setup"<< std::endl;
/*	for (int i=0; i<NUM_CHANNELS; i++) {
		rc[i].set_default_dead_zone(0);
		rc[i].set_range(0,2000);
	}*/
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
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "apm_rc");

  std::string topic_name = std::string("apm_rc");
  //std::cout << topic_name << std::endl;

  ros::NodeHandle n;
  //ros::Publisher baro_pub = n.advertise<apm_baro::apm_baro>(topic_name, 1000);
  //apm_rc::apm_rc msg;

  ros::Rate loop_rate(1);

  setup();
  while (ros::ok()){
	RC_Channel::set_pwm_all();
	
	for (int i=0; i<NUM_CHANNELS; i++) {
		
		int16_t value = rc[i].control_in;
		//std::cout <<"RC "<<i<<" channel value is: "<< (int)value << std::endl;
		printf("ch%u: %4d ", (unsigned)i+1, (int)rc[i].control_in);
		}
	printf("\n");
	
	ros::spinOnce();
	loop_rate.sleep();
  }  
  return 0;
}
