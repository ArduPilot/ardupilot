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

#include "AP_RangeFinder_Bebop.h"

#if AP_RANGEFINDER_BEBOP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <utility>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <linux/types.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <float.h>
#include <math.h>
#include <time.h>
#include <iio.h>
#include <AP_HAL_Linux/Thread.h>
#include <AP_HAL_Linux/GPIO.h>

/*
 * this mode is used at low altitude
 * send 4 wave patterns
 * gpio in low mode
 */
#define RNFD_BEBOP_DEFAULT_MODE 1

/*
 * the number of p7s in the iio buffer
 */
#define RNFD_BEBOP_P7_COUNT 8192

extern const AP_HAL::HAL& hal;

static const uint16_t waveform_mode0[14] = {
    4000, 3800, 3600, 3400, 3200, 3000, 2800,
    2600, 2400, 2200, 2000, 1800, 1600, 1400,
};

static const uint16_t waveform_mode1[32] = {
    4190, 4158, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
    4095, 4090, 4058, 3943, 3924, 3841, 3679, 3588, 3403,
    3201, 3020, 2816, 2636, 2448, 2227, 2111, 1955, 1819,
    1675, 1540, 1492, 1374, 1292
};

AP_RangeFinder_Bebop::AP_RangeFinder_Bebop(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params),
    _thread(NEW_NOTHROW Linux::Thread(FUNCTOR_BIND_MEMBER(&AP_RangeFinder_Bebop::_loop, void)))
{
    _init();
    _freq = RNFD_BEBOP_DEFAULT_FREQ;
    _filtered_capture_size = _adc.buffer_size / _filter_average;
    _filtered_capture = (unsigned int*) calloc(1, sizeof(_filtered_capture[0]) *
            _filtered_capture_size);
    _mode = RNFD_BEBOP_DEFAULT_MODE;
    /* SPI and IIO can not be initialized just yet */
    memset(_tx[0], 0xF0, 16);
    memset(_tx[1], 0xF0, 4);
    memset(_purge, 0xFF, RNFD_BEBOP_NB_PULSES_PURGE);
    _tx_buf = _tx[_mode];
}

AP_RangeFinder_Bebop::~AP_RangeFinder_Bebop()
{
    iio_buffer_destroy(_adc.buffer);
    _adc.buffer = nullptr;
    iio_context_destroy(_iio);
    _iio = nullptr;
}

bool AP_RangeFinder_Bebop::detect()
{
    return true;
}

unsigned short AP_RangeFinder_Bebop::get_threshold_at(int i_capture)
{
    uint16_t threshold_value = 0;

    /*
     * We define several kinds of thresholds signals ; for an echo to be
     * recorded, its maximum amplitude has to be ABOVE that threshold.
     * There is one kind of threshold per mode (mode 0 is "low" and mode 1 is
     * "high")
     * Basically they look like this :
     *
     *            on this part
     *            of the capture
     *  amplitude  we use
     *      ^     the waveform
     *      |     <---------->
     * 4195 +-----+
     *      |
     *      |
     *      |
     *      |
     *  1200|                 +----------------+
     *    +-------------------------------------->
     *      +    low         high               time
     *           limit       limit
     *
     *  */
    switch (_mode) {
    case 0:
        if (i_capture < 139) {
            threshold_value = 4195;
        } else if (i_capture < 153) {
            threshold_value = waveform_mode0[i_capture - 139];
        } else {
            threshold_value = 1200;
        }
        break;

    case 1:
        if (i_capture < 73) {
            threshold_value = 4195;
        } else if (i_capture < 105) {
            threshold_value = waveform_mode1[i_capture - 73];
        } else if (i_capture < 617) {
            threshold_value = 1200;
        } else {
            threshold_value = 4195;
        }
        break;

    default:
        break;
    }

    return threshold_value;
}

int AP_RangeFinder_Bebop::_apply_averaging_filter(void)
{

    int i_filter = 0; /* index in the filtered buffer */
    int i_capture = 0; /* index in the capture buffer : starts incrementing when
                            the captured data first exceeds
                            RNFD_BEBOP_THRESHOLD_ECHO_INIT */
    unsigned int filtered_value = 0;
    bool first_echo = false;
    unsigned char *data;
    unsigned char *start;
    unsigned char *end;
    ptrdiff_t step;

    step = iio_buffer_step(_adc.buffer);
    end = (unsigned char *) iio_buffer_end(_adc.buffer);
    start = (unsigned char *) iio_buffer_first(_adc.buffer, _adc.channel);

    for (data = start; data < end; data += step) {
        unsigned int current_value = 0;
        iio_channel_convert(_adc.channel, &current_value, data);

        /* We keep on advancing in the captured buffer without registering the
         * filtered data until the signal first exceeds a given value */
        if (!first_echo && current_value < threshold_echo_init) {
            continue;
        } else {
            first_echo = true;
        }

        filtered_value += current_value;
        if (i_capture % _filter_average == 0) {
            _filtered_capture[i_filter] = filtered_value / _filter_average;
            filtered_value = 0;
            i_filter++;
        }
        i_capture++;
    }
    return 0;
}

int AP_RangeFinder_Bebop::_search_local_maxima(void)
{
    int i_echo = 0; /* index in echo array */

    for (int i_capture = 1; i_capture <
            (int)_filtered_capture_size - 1; i_capture++) {
        if (_filtered_capture[i_capture] >= get_threshold_at(i_capture)) {
            unsigned short curr = _filtered_capture[i_capture];
            unsigned short prev = _filtered_capture[i_capture - 1];
            unsigned short next = _filtered_capture[i_capture + 1];

            if (curr >= prev && (curr > next || prev <
                        get_threshold_at(i_capture - 1))) {
                _echoes[i_echo].max_index = i_capture;
                i_echo++;
                if (i_echo >= RNFD_BEBOP_MAX_ECHOES) {
                    break;
                }
            }
        }
    }
    _nb_echoes = i_echo;
    return 0;
}

int AP_RangeFinder_Bebop::_search_maximum_with_max_amplitude(void)
{
    unsigned short max = 0;
    int max_idx = -1;

    for (int i_echo = 0; i_echo < _nb_echoes ; i_echo++) {
        unsigned short curr = _filtered_capture[_echoes[i_echo].max_index];
        if (curr > max) {
            max = curr;
            max_idx = i_echo;
        }
    }

    if (max_idx >= 0) {
        return _echoes[max_idx].max_index;
    } else {
        return -1;
    }
}

void AP_RangeFinder_Bebop::_loop(void)
{
    int max_index;

    while(1) {
        _launch();

        _capture();

        if (_apply_averaging_filter() < 0) {
            DEV_PRINTF(
                    "AR_RangeFinder_Bebop: could not apply averaging filter");
        }

        if (_search_local_maxima() < 0) {
            DEV_PRINTF("Did not find any local maximum");
        }

        max_index = _search_maximum_with_max_amplitude();
        if (max_index >= 0) {
            _altitude = (float)(max_index * RNFD_BEBOP_SOUND_SPEED) /
                (2 * (RNFD_BEBOP_DEFAULT_ADC_FREQ / _filter_average));
        }
        _mode = _update_mode(_altitude);
    }
}

void AP_RangeFinder_Bebop::update(void)
{
    static bool first_call = true;

    if (first_call) {
        _thread->start("RangeFinder_Bebop", SCHED_FIFO, 11);
        first_call = false;
    }

    state.distance_m = _altitude;
    state.last_reading_ms = AP_HAL::millis();
    update_status();
}

/*
 * purge is used when changing mode
 */
int AP_RangeFinder_Bebop::_launch_purge()
{
    iio_device_attr_write(_adc.device, "buffer/enable", "1");
    _spi->transfer(_purge, RNFD_BEBOP_NB_PULSES_PURGE, nullptr, 0);
    return 0;
}

void AP_RangeFinder_Bebop::_configure_gpio(int value)
{
    switch (value) {
    case 1: // high voltage
        _gpio->write(LINUX_GPIO_ULTRASOUND_VOLTAGE, 1);
        break;
    case 0: // low voltage
        _gpio->write(LINUX_GPIO_ULTRASOUND_VOLTAGE, 0);
        break;
    default:
        DEV_PRINTF("bad gpio value (%d)", value);
        break;
    }
}

/*
 * reconfigure the pulse that will be sent over spi
 * first send a purge then configure the new pulse
 */
void AP_RangeFinder_Bebop::_reconfigure_wave()
{
    /* configure the output buffer for a purge */
    /* perform a purge */
    if (_launch_purge() < 0) {
        DEV_PRINTF("purge could not send data overspi");
    }
    if (_capture() < 0) {
        DEV_PRINTF("purge could not capture data");
    }

    _tx_buf = _tx[_mode];
    switch (_mode) {
    case 1: /* low voltage */
        _configure_gpio(0);
        break;
    case 0: /* high voltage */
        _configure_gpio(1);
        break;
    default:
        DEV_PRINTF("WARNING, invalid value to configure gpio\n");
        break;
    }
}

/*
 * First configuration of the pulse that will be send over spi
 */
int AP_RangeFinder_Bebop::_configure_wave()
{
    _spi->set_speed(AP_HAL::Device::SPEED_HIGH);
    _configure_gpio(0);
    return 0;
}

/*
 * Configure the adc to get the samples
 */
int AP_RangeFinder_Bebop::_configure_capture()
{
    const char *adcname = "p7mu-adc_2";
    const char *adcchannel = "voltage2";
    /* configure adc interface using libiio */
    _iio = iio_create_local_context();
    if (!_iio) {
        return -1;
    }
    _adc.device = iio_context_find_device(_iio, adcname);

    if (!_adc.device) {
        DEV_PRINTF("Unable to find %s", adcname);
        goto error_destroy_context;
    }
    _adc.channel = iio_device_find_channel(_adc.device, adcchannel,
            false);
    if (!_adc.channel) {
        DEV_PRINTF("Fail to init adc channel %s", adcchannel);
        goto error_destroy_context;
    }

    iio_channel_enable(_adc.channel);

    _adc.freq = RNFD_BEBOP_DEFAULT_ADC_FREQ >> RNFD_BEBOP_FILTER_POWER;
    _adc.threshold_time_rejection = 2.0 / RNFD_BEBOP_SOUND_SPEED *
        _adc.freq;

    /* Create input buffer */
    _adc.buffer_size = RNFD_BEBOP_P7_COUNT;
    if (iio_device_set_kernel_buffers_count(_adc.device, 1)) {
        DEV_PRINTF("cannot set buffer count");
        goto error_destroy_context;
    }
    _adc.buffer = iio_device_create_buffer(_adc.device,
            _adc.buffer_size, false);
    if (!_adc.buffer) {
        DEV_PRINTF("Fail to create buffer : %s", strerror(errno));
        goto error_destroy_context;
    }

    return 0;

error_destroy_context:
    iio_buffer_destroy(_adc.buffer);
    _adc.buffer = nullptr;
    iio_context_destroy(_iio);
    _iio = nullptr;
    return -1;
}

void AP_RangeFinder_Bebop::_init()
{
    _spi = std::move(hal.spi->get_device("bebop"));

    _gpio = AP_HAL::get_HAL().gpio;
    if (_gpio == nullptr) {
        AP_HAL::panic("Could not find GPIO device for Bebop ultrasound");
    }

    if (_configure_capture() < 0) {
        return;
    }

    _configure_wave();

    return;
}

/*
 * enable the capture buffer
 * send a pulse over spi
 */
int AP_RangeFinder_Bebop::_launch()
{
    iio_device_attr_write(_adc.device, "buffer/enable", "1");
    _spi->transfer(_tx_buf, RNFD_BEBOP_NB_PULSES_MAX, nullptr, 0);
    return 0;
}

/*
 * read the iio buffer
 * iio_buffer_refill is blocking by default, so this function is also
 * blocking until samples are available
 * disable the capture buffer
 */
int AP_RangeFinder_Bebop::_capture()
{
    int ret;

    ret = iio_buffer_refill(_adc.buffer);
    iio_device_attr_write(_adc.device, "buffer/enable", "0");
    return ret;
}

int AP_RangeFinder_Bebop::_update_mode(float altitude)
{
    switch (_mode) {
    case 0:
        if (altitude < RNFD_BEBOP_TRANSITION_HIGH_TO_LOW
                && !is_zero(altitude)) {
            if (_hysteresis_counter > RNFD_BEBOP_TRANSITION_COUNT) {
                _mode = 1;
                _hysteresis_counter = 0;
                _reconfigure_wave();
            } else {
                _hysteresis_counter++;
            }
        } else {
            _hysteresis_counter = 0;
        }
        break;

    default:
    case 1:
        if (altitude > RNFD_BEBOP_TRANSITION_LOW_TO_HIGH
                || is_zero(altitude)) {
            if (_hysteresis_counter > RNFD_BEBOP_TRANSITION_COUNT) {
                _mode = 0;
                _hysteresis_counter = 0;
                _reconfigure_wave();
            } else {
                _hysteresis_counter++;
            }
        } else {
            _hysteresis_counter = 0;
        }
        break;
    }
    return _mode;
}

#endif  // AP_RANGEFINDER_BEBOP_ENABLED
