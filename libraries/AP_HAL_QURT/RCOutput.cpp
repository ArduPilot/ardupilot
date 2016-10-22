#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

#include "RCOutput.h"
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <dev_fs_lib_serial.h>

extern const AP_HAL::HAL& hal;

using namespace QURT;

void RCOutput::init()
{
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    // no support for changing frequency yet
}

uint16_t RCOutput::get_freq(uint8_t ch)
{
    // return fixed fake value
    return 490;
}

void RCOutput::enable_ch(uint8_t ch)
{
    if (ch >= channel_count) {
        return;
    }
    enable_mask |= 1U<<ch;
}

void RCOutput::disable_ch(uint8_t ch)
{
    if (ch >= channel_count) {
        return;
    }
    enable_mask &= ~1U<<ch;
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= channel_count) {
        return;
    }
    period[ch] = period_us;
    if (!corked) {
        need_write = true;
    }
}

uint16_t RCOutput::read(uint8_t ch)
{
    if (ch >= channel_count) {
        return 0;
    }
    return period[ch];
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

extern "C" {
    // discard incoming data
    static void read_callback_trampoline(void *, char *, size_t ) {}
}

void RCOutput::timer_update(void)
{
    if (fd == -1 && device_path != nullptr) {
        HAP_PRINTF("Opening RCOutput %s", device_path);
        fd = open(device_path, O_RDWR|O_NONBLOCK);
        if (fd == -1) {
            AP_HAL::panic("Unable to open %s", device_path);
        }
        HAP_PRINTF("Opened ESC UART %s fd=%d\n", device_path, fd);
        if (fd != -1) {
            struct dspal_serial_ioctl_data_rate rate;
            rate.bit_rate = DSPAL_SIO_BITRATE_115200;
            ioctl(fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&rate);

            struct dspal_serial_ioctl_receive_data_callback callback;
            callback.context = this;
            callback.rx_data_callback_func_ptr = read_callback_trampoline;
            ioctl(fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK, (void *)&callback);
        }
    }
    if (!need_write || fd == -1) {
        return;
    }
    struct PACKED {
        uint8_t magic = 0xF7;
        uint16_t period[channel_count];
        uint16_t crc;
    } frame;
    memcpy(frame.period, period, sizeof(period));
    frame.crc = crc_calculate((uint8_t*)frame.period, channel_count*2);
    need_write = false;
    ::write(fd, (uint8_t *)&frame, sizeof(frame));
}

void RCOutput::cork(void)
{
    corked = true;
}

void RCOutput::push(void)
{
    need_write = true;
    corked = false;
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

