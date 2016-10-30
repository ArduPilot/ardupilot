#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>

#include <AP_Common/AP_Common.h>

#include "RCInput_Navio2.h"

using namespace Linux;

extern const AP_HAL::HAL& hal;

#define RCIN_SYSFS_PATH "/sys/kernel/rcio/rcin"

void RCInput_Navio2::init()
{
    for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
        channels[i] = open_channel(i);
        if (channels[i] < 0) {
            AP_HAL::panic("[RCInput_Navio2]: failed to open channels\n");
        }
    }
}

void RCInput_Navio2::_timer_tick(void)
{
    if (AP_HAL::micros() - _last_timestamp < 10000) {
        return;
    }

    char buffer[12];

    for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
        if (::pread(channels[i], buffer, sizeof(buffer) - 1, 0) <= 0) {
            /* We ignore error in order not to spam the console */
            continue;
        }

        buffer[sizeof(buffer) - 1]  = '\0';
        periods[i] = atoi(buffer);
    }

    _update_periods(periods, ARRAY_SIZE(periods));

    _last_timestamp = AP_HAL::micros();
}

RCInput_Navio2::RCInput_Navio2()
{

}

RCInput_Navio2::~RCInput_Navio2()
{
}

int RCInput_Navio2::open_channel(int channel)
{
    char *channel_path;
    if (asprintf(&channel_path, "%s/ch%d", RCIN_SYSFS_PATH, channel) == -1) {
        AP_HAL::panic("[RCInput_Navio2]: not enough memory\n");
    }

    int fd = ::open(channel_path, O_RDONLY|O_CLOEXEC);

    free(channel_path);

    return fd;
}


#endif
