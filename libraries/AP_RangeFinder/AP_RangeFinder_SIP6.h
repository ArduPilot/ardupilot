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

#include <stdint.h>
#include <AP_HAL_Linux/Thread.h>
#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

/*
 * The number of samples in the ADC buffer.
 */
#define RNFD_SIP6_NB_SAMPLES 8192

/*
 * The size of the buffer sent over SPI
 */
#define RNFD_SIP6_NB_PULSES_MAX 32

/*
 * The size of the purge buffer sent over SPI
 */
#define RNFD_SIP6_NB_PULSES_PURGE 64

/*
 * ADC sample frequency
 */
#define RNFD_SIP6_ADC_FREQ (44100 * 3)

/*
 * Speed of sound (m/s)
 */
#define RNFD_SIP6_SOUND_SPEED 340

/* above this altitude we should use high mode */
#define RNFD_SIP6_TRANSITION_HIGH_TO_LOW 1

/* below this altitude we should use low mode */
#define RNFD_SIP6_TRANSITION_LOW_TO_HIGH 1.5

/* count this times before switching mode */
#define RNFD_SIP6_TRANSITION_COUNT 5

/*
 * he number of echoes we will keep at most
 */
#define RNFD_SIP6_MAX_ECHOES 30

struct echo {
    size_t max_index; /* index in the capture buffer at which the maximum is reached */
    size_t distance_index; /* index in the capture buffer at which the signal is for
                              the first time above a fixed threshold below the
                              maximum => this corresponds to the real distance
                              that should be attributed to this echo */
};

class AP_RangeFinder_SIP6 : public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_SIP6(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    ~AP_RangeFinder_SIP6();
    static bool detect();
    void update() override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    enum Mode {
        MODE_HIGH_ALTITUDE = 0,
        MODE_LOW_ALTITUDE
    };

    void _init();
    int _launch();
    int _launch_purge();
    int _launch_data(uint32_t* data, uint8_t len);
    int _capture();
    void _update_mode(float altitude);
    void _configure_gpio(Mode mode);
    void _reconfigure_wave();
    int _spi_transfer(uint32_t *data, uint8_t len);
    void _loop();

    unsigned short get_threshold_at(size_t i_capture);
    void _apply_averaging_filter();
    void _search_local_maxima();
    int _search_maximum_with_max_amplitude();

    Linux::Thread *_thread;

    uint32_t _tx[2][RNFD_SIP6_NB_PULSES_MAX];
    uint32_t _purge[RNFD_SIP6_NB_PULSES_PURGE] = { 0xFF };
    uint32_t* _tx_buf;
    uint16_t _adc_buf[RNFD_SIP6_NB_SAMPLES];
    int _hysteresis_counter;
    const uint16_t threshold_echo_init = 500;
    int _fd = -1;
    uint32_t _last_reading_ms;
    Mode _mode = MODE_LOW_ALTITUDE;
    size_t _nb_echoes;
    float _altitude;
    uint16_t *_filtered_capture;
    size_t _filtered_capture_size;
    struct echo _echoes[RNFD_SIP6_MAX_ECHOES];
    const unsigned int _filter_average = 4;
};

