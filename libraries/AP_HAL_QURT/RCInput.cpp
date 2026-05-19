#include "RCInput.h"
#include <AP_Math/AP_Math.h>

using namespace QURT;

void RCInput::init()
{
#if AP_RCPROTOCOL_ENABLED
    AP::RC().init();
#endif  // AP_RCPROTOCOL_ENABLED
}

bool RCInput::new_input()
{
    bool ret = updated;
    updated = false;
    return ret;
}

uint8_t RCInput::num_channels()
{
    return num_chan;
}

uint16_t RCInput::read(uint8_t channel)
{
    if (channel >= MIN(RC_INPUT_MAX_CHANNELS, num_chan)) {
        return 0;
    }
    return values[channel];
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    WITH_SEMAPHORE(mutex);
    len = MIN(len, num_chan);
    memcpy(periods, values, len*sizeof(periods[0]));
    return len;
}

void RCInput::_timer_tick(void)
{
#if AP_RCPROTOCOL_ENABLED
    auto &rcprot = AP::RC();

    WITH_SEMAPHORE(mutex);

    rcprot.update();

    if (rcprot.new_input()) {
        num_chan = rcprot.num_channels();
        num_chan = MIN(num_chan, RC_INPUT_MAX_CHANNELS);
        rcprot.read(values, num_chan);
        updated = true;
    }
#endif  // AP_RCPROTOCOL_ENABLED
}
