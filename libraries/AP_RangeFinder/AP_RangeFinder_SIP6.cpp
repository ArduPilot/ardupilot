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

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ROLLING_SPIDER

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <AP_HAL_Linux/Scheduler.h>
#include <AP_HAL_Linux/GPIO.h>
#include "AP_RangeFinder_SIP6.h"

#define IOCTL_USND_MAGIC 'u'
#define IOCTL_USND_SETSIZE        _IOW(IOCTL_USND_MAGIC, 1, int)
#define IOCTL_USND_COPYSAMPLE     _IOR(IOCTL_USND_MAGIC, 2, int*)
#define IOCTL_USND_SPI_LEN        _IOW(IOCTL_USND_MAGIC, 3, int)
#define IOCTL_USND_SPI_DAT        _IOW(IOCTL_USND_MAGIC, 4, int)

/*
 * Pin used to select voltage for low or high altitude mode.
 */
#define RNFD_SIP6_VOLTAGE_PIN 56

extern const AP_HAL::HAL& hal;

static const uint16_t waveform_mode_high[14] = {
    1000, 950, 900, 850, 800, 750, 700,
    650,  600, 550, 500, 450, 400, 350,
};

static const uint16_t waveform_mode_low[32] = {
    1010, 1000, 990, 980, 960, 940, 920, 900,
    880,  860,  780, 740, 700, 660, 620, 580,
    540,  500,  480, 460, 440, 420, 400, 390,
    380,  370,  360, 350, 340, 330, 320, 310,
};

AP_RangeFinder_SIP6::AP_RangeFinder_SIP6(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params),
    _thread(new Linux::Thread(FUNCTOR_BIND_MEMBER(&AP_RangeFinder_SIP6::_loop, void)))
{
    _filtered_capture_size = RNFD_SIP6_NB_SAMPLES / _filter_average;
    _filtered_capture = (uint16_t*) calloc(1, sizeof(_filtered_capture[0]) *
                                           _filtered_capture_size);

    // Note: both modes currently use the same pattern, but are kept separate
    // to allow them to be easily modified in case other settings are found to
    // increase performance.
    // At high altitude, send 16 pulses
    for(size_t i = 0; i < 16; ++i) {
        _tx[MODE_HIGH_ALTITUDE][i] = 0xF0;
    }
    // At low altitude, send 16 pulses
    for(size_t i = 0; i < 16; ++i) {
        _tx[MODE_LOW_ALTITUDE][i] = 0xF0;
    }
    _tx_buf = _tx[_mode];

    _init();
}

AP_RangeFinder_SIP6::~AP_RangeFinder_SIP6()
{
    close(_fd);
    _fd = -1;
}

bool AP_RangeFinder_SIP6::detect()
{
    return access("/dev/ultra_snd", R_OK | W_OK) == 0;
}

unsigned short AP_RangeFinder_SIP6::get_threshold_at(size_t i_capture)
{
    uint16_t threshold_value = 0;

    /*
     * The sensor detects its own transmitted pulse at the start of the ADC
     * data. A threshold curve is used to ignore this pulse and only detect
     * reflections. For an echo to be recorded, its maximum amplitude must be
     * above the threshold curve. To get good low altitude performance, the
     * threshold needs to hug as close as possible to the initial pulse to
     * allow it to separated from a fast reflection.
     *
     * Each mode has its own threshold curve, which is defined as a piecewise
     * function that basically looks like this:
     *
     *            on this part
     *            of the capture
     *  amplitude  we use
     *      ^     the waveform
     *      |     <---------->
     * 1024 +-----+
     *      |
     *      |
     *      |
     *      |
     *  200 |                 +----------------+
     *    +-------------------------------------->
     *      +    low         high               sample index
     *           limit       limit
     *
     *  */
    switch (_mode) {
    case MODE_HIGH_ALTITUDE:
        if (i_capture < 139) {
            threshold_value = 1024;
        } else if (i_capture < 139 + ARRAY_SIZE(waveform_mode_high)) {
            threshold_value = waveform_mode_high[i_capture - 139];
        } else {
            threshold_value = 200;
        }
        break;

    case MODE_LOW_ALTITUDE:
        if (i_capture < 50) {
            threshold_value = 1024;
        } else if (i_capture < 50 + ARRAY_SIZE(waveform_mode_low)) {
            threshold_value = waveform_mode_low[i_capture - 50];
        } else if (i_capture < 617) {
            threshold_value = 200;
        } else {
            threshold_value = 1024;
        }
        break;

    default:
        break;
    }

    return threshold_value;
}

void AP_RangeFinder_SIP6::_apply_averaging_filter(void)
{

    size_t i_filter = 0; /* index in the filtered buffer */
    size_t i_capture = 0; /* index in the capture buffer: starts incrementing
                             when the captured data first exceeds
                             RNFD_BEBOP_THRESHOLD_ECHO_INIT */
    uint32_t filtered_value = 0;
    bool first_echo = false;

    for (size_t i = 0; i < RNFD_SIP6_NB_SAMPLES; ++i) {
        uint16_t current_value = _adc_buf[i];

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
}

void AP_RangeFinder_SIP6::_search_local_maxima(void)
{
    size_t i_echo = 0; /* index in echo array */

    for (size_t i_capture = 1; i_capture <
            _filtered_capture_size - 1; ++i_capture) {
        if (_filtered_capture[i_capture] >= get_threshold_at(i_capture)) {
            unsigned short curr = _filtered_capture[i_capture];
            unsigned short prev = _filtered_capture[i_capture - 1];
            unsigned short next = _filtered_capture[i_capture + 1];

            if (curr >= prev && (curr > next || prev <
                                 get_threshold_at(i_capture - 1))) {
                _echoes[i_echo].max_index = i_capture;
                i_echo++;
                if (i_echo >= RNFD_SIP6_MAX_ECHOES) {
                    break;
                }
            }
        }
    }
    _nb_echoes = i_echo;
}

int AP_RangeFinder_SIP6::_search_maximum_with_max_amplitude(void)
{
    unsigned short max = 0;
    int max_idx = -1;

    for (size_t i_echo = 0; i_echo < _nb_echoes ; i_echo++) {
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

void AP_RangeFinder_SIP6::_loop(void)
{
    int max_index;

    while(1) {
        _launch();

        _capture();

        _apply_averaging_filter();

        // for (size_t i = 0; i < _filtered_capture_size; ++i) {
        //     printf("%u,", _filtered_capture[i]);
        // }
        // printf("\n");

        _search_local_maxima();

        max_index = _search_maximum_with_max_amplitude();
        if (max_index >= 0) {
            _altitude = (float)(max_index * RNFD_SIP6_SOUND_SPEED) /
                        (2 * (RNFD_SIP6_ADC_FREQ / _filter_average));
            _last_reading_ms = AP_HAL::millis();
        }
        _update_mode(_altitude);

        Linux::Scheduler::from(hal.scheduler)->microsleep(100 * 1000);
    }
}

void AP_RangeFinder_SIP6::update(void)
{
    static bool first_call = true;

    if (first_call) {
        _thread->start("RangeFinder_SIP6", SCHED_FIFO, 11);
        first_call = false;
    }

    state.distance_cm = (uint16_t) (_altitude * 100);
    state.last_reading_ms = _last_reading_ms;
    update_status();
}

/*
 * Send a pulse over SPI (also enables ADC).
 */
int AP_RangeFinder_SIP6::_launch()
{
    return _launch_data(_tx_buf, RNFD_SIP6_NB_PULSES_MAX);
}

/*
 * Purge is used when changing modes.
 */
int AP_RangeFinder_SIP6::_launch_purge()
{
    return _launch_data(_purge, RNFD_SIP6_NB_PULSES_PURGE);
}

/*
 * Configure ADC and send buffer over SPI.
 */
int AP_RangeFinder_SIP6::_launch_data(uint32_t *data, uint8_t len) {
    // Configure the ADC buffer size and prepare DMA transfer
    // Must be called before every launch for data to be correctly transferred
    uint32_t adc_buf_size = sizeof(_adc_buf);
    int ret = ioctl(_fd, IOCTL_USND_SETSIZE, &adc_buf_size);
    if (ret) {
        return ret;
    }

    return _spi_transfer(data, len);
}

/*
 * Read the ADC buffer.
 */
int AP_RangeFinder_SIP6::_capture()
{
    pollfd fd = {
        .fd = _fd,
        .events = POLLIN
    };
    int ret = poll(&fd, 1, -1);
    if (ret <= 0 || !(fd.revents & POLLIN)) {
        return ret;
    }

    return ioctl(fd.fd, IOCTL_USND_COPYSAMPLE, _adc_buf);
}

/*
 * Send a buffer over SPI. Only the least significant byte of each data element
 * appears to be sent.
 */
int AP_RangeFinder_SIP6::_spi_transfer(uint32_t *data, uint8_t len)
{
    int ret = ioctl(_fd, IOCTL_USND_SPI_LEN, &len);
    if (ret) {
        return ret;
    }
    return ioctl(_fd, IOCTL_USND_SPI_DAT, data);
}

void AP_RangeFinder_SIP6::_configure_gpio(AP_RangeFinder_SIP6::Mode mode)
{
    hal.gpio->pinMode(RNFD_SIP6_VOLTAGE_PIN, HAL_GPIO_OUTPUT);
    switch (mode) {
    case MODE_HIGH_ALTITUDE: // high voltage (3.3V)
        hal.gpio->write(RNFD_SIP6_VOLTAGE_PIN, 0);
        break;
    case MODE_LOW_ALTITUDE: // low voltage (0.8V)
        hal.gpio->write(RNFD_SIP6_VOLTAGE_PIN, 1);
        break;
    }
}

/*
 * Reconfigure the pulse that will be sent over SPI.
 * First send a purge then configure the new pulse.
 */
void AP_RangeFinder_SIP6::_reconfigure_wave()
{
    _launch_purge();
    _capture();

    _tx_buf = _tx[_mode];
    _configure_gpio(_mode);
}

void AP_RangeFinder_SIP6::_init()
{
    _fd = open("/dev/ultra_snd", O_RDWR | O_CLOEXEC);
    if (_fd == -1) {
        AP_HAL::panic("Could not open /dev/ultra_snd: %s", strerror(errno));
    }

    _configure_gpio(_mode);

    return;
}

void AP_RangeFinder_SIP6::_update_mode(float altitude)
{
    switch (_mode) {
    case MODE_HIGH_ALTITUDE:
        if (altitude < RNFD_SIP6_TRANSITION_HIGH_TO_LOW
                && !is_zero(altitude)) {
            if (_hysteresis_counter > RNFD_SIP6_TRANSITION_COUNT) {
                _mode = MODE_LOW_ALTITUDE;
                hal.console->printf("Switch to mode: LOW\n");
                _hysteresis_counter = 0;
                _reconfigure_wave();
            } else {
                _hysteresis_counter++;
            }
        } else {
            _hysteresis_counter = 0;
        }
        break;

    case MODE_LOW_ALTITUDE:
        if (altitude > RNFD_SIP6_TRANSITION_LOW_TO_HIGH
                || is_zero(altitude)) {
            if (_hysteresis_counter > RNFD_SIP6_TRANSITION_COUNT) {
                _mode = MODE_HIGH_ALTITUDE;
                hal.console->printf("Switch to mode: HIGH\n");
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
}
#endif
