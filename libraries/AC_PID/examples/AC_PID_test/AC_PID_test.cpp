/*
 *       Example of PID library.
 *       2012 Code by Jason Short, Randy Mackay. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_HELI_PID.h>
#include <RC_Channel/RC_Channel.h>

// we need a board config created so that the io processor is available
#if HAL_WITH_IO_MCU
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class RC_Channel_PIDTest : public RC_Channel
{
};

class RC_Channels_PIDTest : public RC_Channels
{
public:
    RC_Channel *channel(uint8_t chan) override {
        return &obj_channels[chan];
    }

    RC_Channel_PIDTest obj_channels[NUM_RC_CHANNELS];
private:
    int8_t flight_mode_channel_number() const override { return -1; };
};

#define RC_CHANNELS_SUBCLASS RC_Channels_PIDTest
#define RC_CHANNEL_SUBCLASS RC_Channel_PIDTest

#include <RC_Channel/RC_Channels_VarInfo.h>

RC_Channels_PIDTest _rc;

// default PID values
#define TEST_P 1.0f
#define TEST_I 0.01f
#define TEST_D 0.2f
#define TEST_IMAX 10
#define TEST_FILTER 5.0f
#define TEST_DT 0.01f
#define TEST_INITIAL_FF 0.0f

// setup function
void setup()
{
    hal.console->printf("ArduPilot AC_PID library test\n");

#if HAL_WITH_IO_MCU
    BoardConfig.init();
#endif

    rc().init();

    hal.scheduler->delay(1000);
}

// main loop
void loop()
{
    // setup (unfortunately must be done here as we cannot create a global AC_PID object)
    AC_PID *pid = new AC_PID(TEST_P, TEST_I, TEST_D, 0.0f, TEST_IMAX * 100.0f, 0.0f, 0.0f, TEST_FILTER);
    // AC_HELI_PID *heli_pid= new AC_HELI_PID(TEST_P, TEST_I, TEST_D, TEST_INITIAL_FF, TEST_IMAX * 100, 0.0f, 0.0f, TEST_FILTER);

    // display PID gains
    hal.console->printf("P %f  I %f  D %f  imax %f\n", (double)pid->kP(), (double)pid->kI(), (double)pid->kD(), (double)pid->imax());

    RC_Channel *c = rc().channel(0);
    if (c == nullptr) {
        while (true) {
            hal.console->printf("No channel 0?");
            hal.scheduler->delay(1000);
        }
    }

    // capture radio trim
    const uint16_t radio_trim = c->get_radio_in();

    while (true) {
        rc().read_input(); // poll the radio for new values
        const uint16_t radio_in = c->get_radio_in();
        const int16_t error = radio_in - radio_trim;
        pid->update_error(error, TEST_DT);
        const float control_P = pid->get_p();
        const float control_I = pid->get_i();
        const float control_D = pid->get_d();

        // display pid results
        hal.console->printf("radio: %d\t err: %d\t pid:%4.2f (p:%4.2f i:%4.2f d:%4.2f)\n",
                (int)radio_in, (int)error,
                (double)(control_P+control_I+control_D),
                (double)control_P, (double)control_I, (double)control_D);
        hal.scheduler->delay(50);
    }
}

AP_HAL_MAIN();
