/*
  simple test of RC input interface
 */
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define MAX_CHANNELS 16

static uint8_t max_channels_display = 8;  // Set to 0 for display numbers of channels detected.
static uint16_t last_value[MAX_CHANNELS];

void setup(void)
{
    hal.console->printf("Starting RCInput test\n");
}

void loop(void)
{
    bool changed = false;
    uint8_t nchannels = hal.rcin->num_channels();  // Get the numbers channels detected by RC_INPUT.
    if (nchannels > MAX_CHANNELS) {
        nchannels = MAX_CHANNELS;
    }
    if (nchannels != 0) {  // If channels detected, process.
        for (uint8_t i = 0; i < nchannels; i++) {
            uint16_t v = hal.rcin->read(i);
            if (last_value[i] != v) {
                changed = true;
                last_value[i] = v;
            }
        }
        if (max_channels_display > nchannels) {
            max_channels_display = nchannels;
        }
        if (max_channels_display > 0) {
            if (changed) {
                for (uint8_t i = 0; i < max_channels_display; i++) {
                    hal.console->printf("%2u:%04u ", (unsigned)i+1, (unsigned)last_value[i]);
                }
                hal.console->printf("\n");
            }
        } else {
            hal.console->printf("Channels detected: %2u\n", nchannels);
            hal.console->printf("Set max_channels_display > 0 to display channels values\n");
        }
    }
    hal.scheduler->delay(100);
}

AP_HAL_MAIN();
