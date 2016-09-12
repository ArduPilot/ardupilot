#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include <stdio.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <dev_fs_lib_serial.h>

#include "RCInput.h"
#include <AP_HAL/utility/dsm.h>

extern const AP_HAL::HAL& hal;

using namespace QURT;

RCInput::RCInput(const char *_device_path) :
    device_path(_device_path),
    new_rc_input(false)
{
}

extern "C" {
static void read_callback_trampoline(void *, char *, size_t );
}

void RCInput::init()
{
    if (device_path == nullptr) {
        return;
    }
    fd = open(device_path, O_RDONLY|O_NONBLOCK);
    if (fd == -1) {
        AP_HAL::panic("Unable to open RC input %s", device_path);
    }

    struct dspal_serial_ioctl_data_rate rate;
    rate.bit_rate = DSPAL_SIO_BITRATE_115200;
    int ret = ioctl(fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&rate);

    struct dspal_serial_ioctl_receive_data_callback callback;
    callback.context = this;
    callback.rx_data_callback_func_ptr = read_callback_trampoline;
    ret = ioctl(fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK, (void *)&callback);
}

static void read_callback_trampoline(void *ctx, char *buf, size_t size)
{
    ((RCInput *)ctx)->read_callback(buf, size);
}

/*
  callback for incoming data
 */
void RCInput::read_callback(char *buf, size_t size)
{
    add_dsm_input((const uint8_t *)buf, size);
}

bool RCInput::new_input() 
{
    return new_rc_input;
}

uint8_t RCInput::num_channels() 
{
    return _num_channels;
}

uint16_t RCInput::read(uint8_t ch) 
{
    new_rc_input = false;
    if (_override[ch]) {
        return _override[ch];
    }
    if (ch >= _num_channels) {
        return 0;
    }
    return _pwm_values[ch];
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len) 
{
    uint8_t i;
    for (i=0; i<len; i++) {
        if((periods[i] = read(i))){
            continue;
        }
        else{
            break;
        }
    }
    return (i+1);
}

bool RCInput::set_overrides(int16_t *overrides, uint8_t len) 
{
    bool res = false;
    if(len > QURT_RC_INPUT_NUM_CHANNELS){
        len = QURT_RC_INPUT_NUM_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool RCInput::set_override(uint8_t channel, int16_t override) 
{
    if (override < 0) return false; /* -1: no change. */
    if (channel < QURT_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            new_rc_input = true;
            return true;
        }
    }
    return false;
}

void RCInput::clear_overrides()
{
    for (uint8_t i = 0; i < QURT_RC_INPUT_NUM_CHANNELS; i++) {
       _override[i] = 0;
    }
}


/*
  add some bytes of input in DSM serial stream format, coping with partial packets
 */
void RCInput::add_dsm_input(const uint8_t *bytes, size_t nbytes)
{
    if (nbytes == 0) {
        return;
    }
    const uint8_t dsm_frame_size = sizeof(dsm.frame);

    uint32_t now = AP_HAL::millis();    
    if (now - dsm.last_input_ms > 5) {
        // resync based on time
        dsm.partial_frame_count = 0;
    }
    dsm.last_input_ms = now;
    
    while (nbytes > 0) {
        size_t n = nbytes;
        if (dsm.partial_frame_count + n > dsm_frame_size) {
            n = dsm_frame_size - dsm.partial_frame_count;
        }
        if (n > 0) {
            memcpy(&dsm.frame[dsm.partial_frame_count], bytes, n);
            dsm.partial_frame_count += n;
            nbytes -= n;
            bytes += n;
        }

	if (dsm.partial_frame_count == dsm_frame_size) {
            dsm.partial_frame_count = 0;
            uint16_t values[16] {};
            uint16_t num_values=0;
            if (dsm_decode(AP_HAL::micros64(), dsm.frame, values, &num_values, 16) &&
                num_values >= 5) {
                for (uint8_t i=0; i<num_values; i++) {
                    if (values[i] != 0) {
                        _pwm_values[i] = values[i];
                    }
                }
                /*
                  the apparent number of channels can change on DSM,
                  as they are spread across multiple frames. We just
                  use the max num_values we get
                 */
                if (num_values > _num_channels) {
                    _num_channels = num_values;
                }
                new_rc_input = true;
#if 0
                HAP_PRINTF("Decoded DSM %u channels %u %u %u %u %u %u %u %u\n",
                           (unsigned)num_values,
                           values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7]);
#endif
            }
        }
    }
}

#endif // CONFIG_HAL_BOARD
