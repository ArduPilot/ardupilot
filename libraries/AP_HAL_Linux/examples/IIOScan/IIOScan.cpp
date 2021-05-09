
#include <AP_HAL/AP_HAL.h>
#include <cstdio>
#include <cstdint>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if defined(HAVE_LIBIIO)
#include <iio.h>
void setup()
{
    hal.console->println("IIOScan startup...\n");
    uint32_t major, minor;
    char git_tag[8];
    iio_library_get_version(&major, &minor, git_tag);
    printf("Library version: %u.%u (git tag: %s)\n", major, minor, git_tag);
}

void loop()
{
    const struct iio_context *ctx = iio_create_local_context();
    if (!ctx) {
        printf("Failed to create context \n");
        exit(1);
    }
    const auto device_count = iio_context_get_devices_count(ctx);
    printf("Found %d devices !\n", device_count);
    for (uint8_t i=0; i< device_count; i++) {
        const struct iio_device *dev = iio_context_get_device(ctx, i);

        const char *name = iio_device_get_name(dev);
        if (!name) {
            name = iio_device_get_id(dev);
        }
        printf("Device %d : name : %s\n", i, name);

        const auto channel_count = iio_device_get_channels_count(dev);
        printf("Found %d channels !\n", channel_count);

        for (uint8_t j = 0; j < channel_count; ++j) {
            const struct iio_channel *chn = iio_device_get_channel(dev, j);
            const char *cname = iio_channel_get_name(chn);
            if (!cname) {
                cname = iio_channel_get_id(chn);
            }
            printf("Channel %d : name : %s\n", j, cname);
        }
    }
    hal.scheduler->delay(2000);
    printf("\n\n");
}
#else
void setup(void)
{
    hal.console->println("You need libiio support !\n");
}
void loop(void)
{
    exit(1);
}
#endif
AP_HAL_MAIN();
