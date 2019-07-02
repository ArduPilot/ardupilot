/*
 *       Example of RC_Channel library.
 *       Based on original sketch by Jason Short. 2010
 */

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

// we need a boardconfig created so that the io processor is available
#if HAL_WITH_IO_MCU
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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

    int8_t flight_mode_channel_number() const override { return 5; }

private:

};

#define RC_CHANNELS_SUBCLASS RC_Channels_Example
#define RC_CHANNEL_SUBCLASS RC_Channel_Example

#include <RC_Channel/RC_Channels_VarInfo.h>

static RC_Channels_Example rc_channels;

static void print_radio_values();

#define RC_CHANNELS_TO_DISPLAY 8

void setup()
{
    hal.console->printf("ArduPilot RC Channel test\n");

#if HAL_WITH_IO_MCU
    BoardConfig.init();
#endif

    rc().init();

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
    static uint8_t count = 0;

    if (count++ == 0) {
        for (int i=0; i<RC_CHANNELS_TO_DISPLAY; i++) {
            hal.console->printf("Ch %02d ", (unsigned)i+1);
        }
        hal.console->printf("\n");
    }

    rc().read_input();
    for (uint8_t i=0; i<RC_CHANNELS_TO_DISPLAY; i++) {
	    hal.console->printf("%5d ", (int)rc().channel(i)->get_control_in());
	    // hal.console->printf("%4d ", (int)rc().channel(i)->percent_input());
    }
    hal.console->printf("\n");

    hal.scheduler->delay(20);
}


static void print_radio_values()
{
    for (int i=0; i<RC_CHANNELS_TO_DISPLAY; i++) {
	     hal.console->printf("CH%u: %u|%u\n",
			  (unsigned)i+1,
              (unsigned)rc().channel(i)->get_radio_min(),
			  (unsigned)rc().channel(i)->get_radio_max());
    }
}


AP_HAL_MAIN();
