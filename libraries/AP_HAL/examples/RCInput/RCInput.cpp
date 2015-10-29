/*
  simple test of RC input interface
 */
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define MAX_CHANNELS 16

static uint8_t max_channels = 0;
static uint16_t last_value[MAX_CHANNELS];

void setup(void) 
{
    hal.console->printf("Starting RCInput test\n");
}

void loop(void) 
{
    bool changed = false;
    uint8_t nchannels = hal.rcin->num_channels();
    if (nchannels > MAX_CHANNELS) {
        nchannels = MAX_CHANNELS;
    }
    for (uint8_t i=0; i<nchannels; i++) {
        uint16_t v = hal.rcin->read(i);
        if (last_value[i] != v) {
            changed = true;
            last_value[i] = v;
        }
        if (i > max_channels) {
            max_channels = i;
        }
    }
    if (changed) {
        for (uint8_t i=0; i<max_channels; i++) {
            hal.console->printf("%2u:%04u ", (unsigned)i+1, (unsigned)last_value[i]);
        }
        hal.console->println();
    }
    hal.scheduler->delay(100);
}

AP_HAL_MAIN();
