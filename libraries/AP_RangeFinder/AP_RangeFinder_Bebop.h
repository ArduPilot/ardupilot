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
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL_Linux/Thread.h>

/*
 * the size of the buffer sent over spi
 */
#define RNFD_BEBOP_NB_PULSES_MAX 32

/*
 * the size of the purge buffer sent over spi
 */
#define RNFD_BEBOP_NB_PULSES_PURGE 64

/*
 * default us frequency
 * 17 times by seconds
 */
#define RNFD_BEBOP_DEFAULT_FREQ 17

/*
 * default adc frequency
 */
#define RNFD_BEBOP_DEFAULT_ADC_FREQ 160000

/*
 * to filter data we make the average of (1 << this_value) datas
 */
#define RNFD_BEBOP_FILTER_POWER 2

/*
 * Speed of sound
 */
#define RNFD_BEBOP_SOUND_SPEED 340

/* above this altitude we should use mode 0 */
#define RNFD_BEBOP_TRANSITION_HIGH_TO_LOW 0.75

/* below this altitude we should use mode 1 */
#define RNFD_BEBOP_TRANSITION_LOW_TO_HIGH 1.5

/* count this times before switching mode */
#define RNFD_BEBOP_TRANSITION_COUNT 5

/*
 * the number of echoes we will keep at most
 */
#define RNFD_BEBOP_MAX_ECHOES 30

struct echo {
    int max_index; /* index in the capture buffer at which the maximum is reached */
    int distance_index; /* index in the capture buffer at which the signal is for
                            the first time above a fixed threshold below the
                            maximum => this corresponds to the real distance
                            that should be attributed to this echo */
};

/*
 * struct related to adc
 * data to receive and process adc datas
 */
struct adc_capture {
    struct iio_device *device;
    struct iio_buffer *buffer;
    unsigned int buffer_size;
    struct iio_channel *channel;
    unsigned int freq;

     /* Used in order to match two echoes of two ADC acquisitions */
    unsigned short threshold_time_rejection;
};

class AP_RangeFinder_Bebop : public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_Bebop(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    ~AP_RangeFinder_Bebop(void);
    static bool detect();
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    void _init(void);
    int _launch(void);
    int _capture(void);
    int _update_mode(float altitude);
    void _configure_gpio(int value);
    int _configure_wave();
    void _reconfigure_wave();
    int _configure_capture();
    int _launch_purge();
    void _loop(void);

    Linux::Thread *_thread;
    unsigned short get_threshold_at(int i_capture);
    int _apply_averaging_filter(void);
    int _search_local_maxima(void);
    int _search_maximum_with_max_amplitude(void);

    AP_HAL::OwnPtr<AP_HAL::Device> _spi;
    AP_HAL::GPIO *_gpio;

    struct adc_capture _adc;
    struct iio_context *_iio;

    unsigned char _tx[2][RNFD_BEBOP_NB_PULSES_MAX];
    unsigned char _purge[RNFD_BEBOP_NB_PULSES_PURGE];
    unsigned char* _tx_buf;
    int _hysteresis_counter;
    const unsigned int threshold_echo_init = 1500;
    int _fd = -1;
    uint64_t _last_timestamp;
    int _mode;
    int _nb_echoes;
    int _freq;
    float _altitude;
    unsigned int *_filtered_capture;
    unsigned int _filtered_capture_size;
    struct echo _echoes[RNFD_BEBOP_MAX_ECHOES];
    unsigned int _filter_average = 4;
    int16_t _last_max_distance_cm = 850;
    int16_t _last_min_distance_cm = 32;
};

