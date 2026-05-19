#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_EMLID_RCIO_ENABLED

#include "AP_RCProtocol_Emlid_RCIO.h"

#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>

#include <AP_Common/AP_Common.h>

#define RCIN_SYSFS_PATH "/sys/kernel/rcio/rcin"

void AP_RCProtocol_Emlid_RCIO::init()
{
    for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
        channels[i] = open_channel(i);
        if (channels[i] < 0) {
            AP_HAL::panic("AP_RCProtocol_Emlid_RCIO: failed to open channels");
        }
    }
}

void AP_RCProtocol_Emlid_RCIO::update(void)
{
    if (AP_HAL::micros() - _last_timestamp < 10000) {
        return;
    }

    if (!init_done) {
        init();
        init_done = true;
    }

    uint16_t periods[CHANNEL_COUNT]{};

    char buffer[12];
    for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
        if (::pread(channels[i], buffer, sizeof(buffer) - 1, 0) <= 0) {
            /* We ignore error in order not to spam the console */
            continue;
        }
        buffer[sizeof(buffer) - 1]  = '\0';
        periods[i] = atoi(buffer);
    }

    add_input(ARRAY_SIZE(periods), periods, false);

    _last_timestamp = AP_HAL::micros();
}

int AP_RCProtocol_Emlid_RCIO::open_channel(int channel)
{
    char *channel_path;
    if (asprintf(&channel_path, "%s/ch%d", RCIN_SYSFS_PATH, channel) == -1) {
        AP_HAL::panic("AP_RCProtocol_Emlid_RCIO: not enough memory");
    }

    int fd = ::open(channel_path, O_RDONLY|O_CLOEXEC);

    free(channel_path);

    return fd;
}

#endif  // AP_RCPROTOCOL_EMLID_RCIO_ENABLED
