#pragma once

#include "RCInput.h"


namespace Linux {

class RCInput_Navio2 : public RCInput {
public:
    void init() override;
    void _timer_tick(void) override;
    RCInput_Navio2();
    ~RCInput_Navio2();

private:
    int open_channel(int ch);

    uint64_t _last_timestamp = 0l;
    static const size_t CHANNEL_COUNT = 16;
    int channels[CHANNEL_COUNT];
    uint16_t periods[ARRAY_SIZE(channels)] = {0};
};

}
