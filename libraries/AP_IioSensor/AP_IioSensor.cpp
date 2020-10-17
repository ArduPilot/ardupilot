/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_IioSensor.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <errno.h>
#include <stdio.h>
#include <string.h>


AP_Iio_Channel::AP_Iio_Channel(const char *name) :
    _name(name)
{}

int AP_Iio_Channel::init(struct iio_device *iio_dev, struct iio_buffer *iio_buf)
{
    if (!iio_dev) {
        return -1;
    }

    _iio_chan = iio_device_find_channel(iio_dev, _name, false);
    if (!_iio_chan) {
        fprintf(stderr, "AP_Iio_Channel : failed to lookup for chan %s\n", _name);
        return -1;
    }

    if (!iio_channel_is_scan_element(_iio_chan)) {
        fprintf(stderr,"AP_Iio_Channel : %s cannot read samples\n", _name);
        return -1;
    }

    iio_channel_enable(_iio_chan);
    if (!iio_channel_is_enabled(_iio_chan)) {
        fprintf(stderr,"AP_Iio_Channel : %s is not enabled\n", _name);
        return -1;
    }

    _iio_fmt = iio_channel_get_data_format(_iio_chan);
    if (!_iio_fmt) {
        fprintf(stderr,"AP_Iio_Channel : %s failed to get data format\n", _name);
        return -1;
    }
//    printf("FMT %s: length %u, bits %u, shift %u, is_signed %d, is_be %d, with_scale %d, scale %f, repeat %d\n",
//            _name, _iio_fmt->length, _iio_fmt->bits, _iio_fmt->shift, _iio_fmt->is_signed, _iio_fmt->is_be, _iio_fmt->with_scale, _iio_fmt->scale, _iio_fmt->repeat);

    iio_channel_attr_read_double(_iio_chan, "scale", &_scale);
    iio_channel_attr_read_double(_iio_chan, "offset", &_offset);
    _iio_buf = iio_buf;
    printf("AP_Iio_Channel: added channel %s\n", _name);

    return 0;
}

void AP_Iio_Channel::disable()
{
    iio_channel_disable(_iio_chan);
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-arith"
int AP_Iio_Channel::get_data(double *values) const
{
    int ret = 0;
    unsigned int i;
    void *ptr;
    double value;

    if (!_iio_buf) {
        fprintf(stderr,"AP_Iio_Channel %s: cannot get data with no buffer\n", _name);
        return -1;
    }

    for (i = 0, ptr = iio_buffer_first(_iio_buf, _iio_chan);
         ptr < iio_buffer_end(_iio_buf);
         ptr += iio_buffer_step(_iio_buf), i++) {
        if (_iio_fmt->length == 8 && !_iio_fmt->is_signed) {
            uint8_t val;
            iio_channel_convert(_iio_chan, &val, ptr);
            value = val;
        } else if (_iio_fmt->length == 16 && _iio_fmt->is_signed) {
            int16_t val;
            iio_channel_convert(_iio_chan, &val, ptr);
            value = val;
        } else if (_iio_fmt->length == 16 && !_iio_fmt->is_signed) {
            uint16_t val;
            iio_channel_convert(_iio_chan, &val, ptr);
            value = val;
        } else if ((_iio_fmt->length == 24) && !_iio_fmt->is_signed &&
                   (_iio_fmt->bits == 24)) {
            uint32_t val = 0;
            iio_channel_convert(_iio_chan, &val, ptr);
            /* 24 bits to be transformed into 32bits
             * a zero will be put in the last byte.
             * - In little endian, the last byte will be the msb, so nothing to
             * do here.
             * - In big endian, the last byte will be the lsb, and all other
             *   bytes will be shifted so shift right by 8 bits
             */
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
            val = val >> 8;
#endif
            value = val;
        } else if ((_iio_fmt->length == 32) && _iio_fmt->is_signed &&
                   (_iio_fmt->bits == 24)) {
            uint32_t val;
            iio_channel_convert(_iio_chan, &val, ptr);
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
            val = val >> 8;
#endif
            value = val;
        } else {
            fprintf(stderr,"%s: unhandled channel format length : %d, bits : %d, %s\n", _name,
                    _iio_fmt->length, _iio_fmt->bits, _iio_fmt->is_signed ? "signed" : "unsigned");
            ret = -1;
            continue;
        }
        values[i] = (value + _offset) * _scale;
        //printf("values[i] %f = (value %f + _offset %f) * _scale %f \n", values[i], value, _offset, _scale);
    }

    return ret;
}
#pragma GCC diagnostic pop
AP_Iio_Sensor::AP_Iio_Sensor(const char *name, vector<AP_Iio_Channel*> *channels,
                             long long sampling_freq, unsigned int buf_count, const char* trigger_name, uint8_t kernel_buffer_count)
{
    if (!name || !buf_count) {
        fprintf(stderr,"AP_Iio_Sensor: wrong input parameters\n");
        return;
    }

    _buf_count = buf_count;
    _name = name;
    _kernel_buffer_count = kernel_buffer_count;

    _iio_ctx = iio_create_local_context();
    if (!_iio_ctx) {
        fprintf(stderr,"AP_Iio_Sensor: failed to create iio context for dev %s\n", name);
        return;
    }

    _iio_dev = iio_context_find_device(_iio_ctx, name);
    if (!_iio_dev) {
        fprintf(stderr,"AP_Iio_Sensor: couldn't find device %s\n", name);
        destroy_context();
    }

    int ret = iio_device_attr_write(_iio_dev, "buffer/enable", "0");
    if (ret < 0) {
        fprintf(stderr, "AP_Iio_Sensor: %s failed to write buffer/enable %s\n", _name, strerror(-ret));
        destroy_context();
    }

    if (sampling_freq > 0) {
        ret = iio_device_attr_write_longlong(_iio_dev, "sampling_frequency", sampling_freq);
        if (ret < 0) {
            fprintf(stderr,"AP_Iio_Sensor: %s failed to write sampling frequency %s\n", name, strerror(errno));
            destroy_context();
        }

        ret = iio_device_attr_read_double(_iio_dev, "sampling_frequency", &_sampling_freq);
        if (ret < 0) {
            fprintf(stderr,"AP_Iio_Sensor: %s failed to read sampling frequency %s\n", name, strerror(errno));
            destroy_context();
        }

        printf("AP_Iio_Sensor: %s real sampling freq : %f\n", name, _sampling_freq);
    }

    if (trigger_name) {
        if(!set_trigger(trigger_name)) {
            destroy_context();
        }
    }

    _channels = channels;
}

void AP_Iio_Sensor::destroy_context()
{
    iio_context_destroy(_iio_ctx);
}

AP_Iio_Sensor::~AP_Iio_Sensor()
{
    for (AP_Iio_Channel* chan : *_channels) {
        chan->disable();
    }
    iio_buffer_destroy(_iio_buf);
    iio_context_destroy(_iio_ctx);
}

int AP_Iio_Sensor::init()
{
    if (!_iio_dev) {
        fprintf(stderr,"AP_Iio_Sensor: iio devices not set\n");
        return -1;
    }

    for (AP_Iio_Channel* chan : *_channels) {
        if (chan->init(_iio_dev, _iio_buf) < 0) {
            fprintf(stderr,"failed to init channel %s\n", chan->get_name());
            return -1;
        }
    }
    int ret;
    if (_kernel_buffer_count > 0) {
        ret = iio_device_set_kernel_buffers_count(_iio_dev, _kernel_buffer_count);
        if (ret < 0) {
            fprintf(stderr,"AP_Iio_Sensor: %s failed to set _kernel_buffer_count %s\n", _name, strerror(-ret));
        }
    }
    _iio_buf = iio_device_create_buffer(_iio_dev, _buf_count, false);
    if (!_iio_buf) {
        fprintf(stderr,"AP_Iio_Sensor: failed to create buffer\n");
        return -1;
    }

    for (AP_Iio_Channel* chan : *_channels) {
        chan->set_buf(_iio_buf);
    }

    bool buffer_enable = false;
    ret = iio_device_attr_read_bool(_iio_dev, "buffer/enable", &buffer_enable);
    if (ret < 0) {
        fprintf(stderr,"AP_Iio_Sensor: %s failed to read buffer/enable %s\n", _name, strerror(errno));
        destroy_context();
    }

    if (!buffer_enable) {
        ret = iio_device_attr_write(_iio_dev, "buffer/enable", "1");
        if (ret < 0) {
            fprintf(stderr,"AP_Iio_Sensor: %s failed to write buffer/enable %s\n", _name, strerror(-ret));
            destroy_context();
        }
    }
    auto flushed = buffer_flush();
    fprintf(stderr,"AP_Iio_Sensor: flushed %d\n", flushed);

    return 0;
}

int AP_Iio_Sensor::read() const
{
    const ssize_t ret = iio_buffer_refill(_iio_buf);
    if (ret < 0) {
        fprintf(stderr, "%s iio_buffer_refill error %s\n", _name, strerror(-ret));
        return -1;
    }

    return _buf_count;
}

bool AP_Iio_Sensor::set_trigger(const char *name) {
    const struct iio_device *trig;
    int ret = iio_device_get_trigger(_iio_dev, &trig);
    if (ret == 0) {
        if (trig == nullptr) {
            assign_trigger(name);
        } else {
            const char *trigger_name = iio_device_get_name(trig);
            if (strcmp(trigger_name, name) != 0) {
                assign_trigger(name);
            } else {
                _iio_trigger = trig;
            }
        }
    } else if (ret == -ENOENT) {
        assign_trigger(name);
    } else if (ret < 0) {
        fprintf(stderr, "%s failed to get trigger %s : %s\n", _name, name, strerror(-ret));
        return false;
    }
    return true;
}

bool AP_Iio_Sensor::assign_trigger(const char *name) {
    _iio_trigger = iio_context_find_device(_iio_ctx, name);
    if (!_iio_trigger || !iio_device_is_trigger(_iio_trigger)) {
        fprintf(stderr, "%s unable to use %s as trigger\n", _name, name);
        return false;
    }

    int ret = iio_device_set_trigger(_iio_dev, _iio_trigger);
    if (ret < 0) {
        fprintf(stderr, "%s failed to set trigger %s : %s\n", _name, name, strerror(-ret));
        return false;
    }
    return true;
}

int AP_Iio_Sensor::buffer_flush() {
        ssize_t nsamples = 0;

        int ret = iio_buffer_set_blocking_mode(_iio_buf, false);
        if (ret < 0) {
            fprintf(stderr, "%s failed to set buffer blocking mode %s\n", _name, strerror(-ret));
            return ret;
        }

        do {
            ret = iio_buffer_refill(_iio_buf);
            if (ret < 0 && ret != -EAGAIN) {
                fprintf(stderr, "%s iio_buffer_refill error %s\n", _name, strerror(-ret));
                return ret;
            } else if (ret > 0) {
                nsamples++;
            }
        } while (ret > 0);

        return nsamples;
}
#endif
