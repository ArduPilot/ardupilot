/*
 *       Example of RC_Channel library.
 *       Based on original sketch by Jason Short. 2010
 */

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define NUM_CHANNELS 8

class RC_Channel_Example : public RC_Channel
{
};

class RC_Channels_Example : public RC_Channels
{
public:

    RC_Channel_Example obj_channels[NUM_RC_CHANNELS];

    RC_Channel_Example *channel(const uint8_t chan) override {
        if (chan > NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    int8_t flight_mode_channel_number() const { return 5; }

private:

};

const AP_Param::GroupInfo RC_Channels::var_info[] = {
    // @Group: 1_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[0], "1_",  1, RC_Channels_Example, RC_Channel_Example),

    // @Group: 2_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[1], "2_",  2, RC_Channels_Example, RC_Channel_Example),

    // @Group: 3_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[2], "3_",  3, RC_Channels_Example, RC_Channel_Example),

    // @Group: 4_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[3], "4_",  4, RC_Channels_Example, RC_Channel_Example),

    // @Group: 5_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[4], "5_",  5, RC_Channels_Example, RC_Channel_Example),

    // @Group: 6_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[5], "6_",  6, RC_Channels_Example, RC_Channel_Example),

    // @Group: 7_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[6], "7_",  7, RC_Channels_Example, RC_Channel_Example),

    // @Group: 8_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[7], "8_",  8, RC_Channels_Example, RC_Channel_Example),

    // @Group: 9_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[8], "9_",  9, RC_Channels_Example, RC_Channel_Example),

    // @Group: 10_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[9], "10_", 10, RC_Channels_Example, RC_Channel_Example),

    // @Group: 11_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[10], "11_", 11, RC_Channels_Example, RC_Channel_Example),

    // @Group: 12_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[11], "12_", 12, RC_Channels_Example, RC_Channel_Example),

    // @Group: 13_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[12], "13_", 13, RC_Channels_Example, RC_Channel_Example),

    // @Group: 14_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[13], "14_", 14, RC_Channels_Example, RC_Channel_Example),

    // @Group: 15_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[14], "15_", 15, RC_Channels_Example, RC_Channel_Example),

    // @Group: 16_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[15], "16_", 16, RC_Channels_Example, RC_Channel_Example),

    AP_GROUPEND
};

static RC_Channels_Example rc_channels;

static void print_pwm(void);
static void print_radio_values();


void setup()
{
    hal.console->printf("ArduPilot RC Channel test\n");

    print_radio_values();

    // set type of output, symmetrical angles or a number range;
    rc().channel(CH_1)->set_angle(4500);
    rc().channel(CH_1)->set_default_dead_zone(80);

    rc().channel(CH_2)->set_angle(4500);
    rc().channel(CH_2)->set_default_dead_zone(80);

    rc().channel(CH_3)->set_range(1000);
    rc().channel(CH_3)->set_default_dead_zone(20);

    rc().channel(CH_4)->set_angle(6000);
    rc().channel(CH_4)->set_default_dead_zone(500);

    rc().channel(CH_5)->set_range(1000);

    rc().channel(CH_6)->set_range(800);

    rc().channel(CH_7)->set_range(1000);

    rc().channel(CH_8)->set_range(1000);
}

void loop()
{
    rc().read_input();
    print_pwm();
    
    hal.scheduler->delay(20);
}

static void print_pwm(void)
{
    for (int i=0; i<NUM_CHANNELS; i++) {
	    hal.console->printf("ch%u: %4d ", (unsigned)i+1, (int)rc().channel(i)->get_control_in());
    }
    hal.console->printf("\n");
}


static void print_radio_values()
{
    for (int i=0; i<NUM_CHANNELS; i++) {
	     hal.console->printf("CH%u: %u|%u\n",
			  (unsigned)i+1,
              (unsigned)rc().channel(i)->get_radio_min(),
			  (unsigned)rc().channel(i)->get_radio_max());
    }
}


AP_HAL_MAIN();
