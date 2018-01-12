/*
 * Copyright (C) Siddharth Bharat Purohit 2017
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ppm.h"

#if HAL_USE_ICU

static ICUConfig icucfg;    //Input Capture Unit Config
static uint16_t ppm_buffer[10] = {0};
static bool updated[10] = {0};
static bool available;
static uint8_t buf_ptr = 0;
static uint8_t num_channels = 0;
static void ppm_measurement_cb(ICUDriver*);

//Initiallise ppm ICU with requested configuration
bool ppm_init(uint32_t freq, bool active_high)
{
    icumode_t ppm_active_mode;

    if (active_high) {
        ppm_active_mode = ICU_INPUT_ACTIVE_HIGH;
    } else {
        ppm_active_mode = ICU_INPUT_ACTIVE_LOW;
    }

    icucfg.mode = ppm_active_mode;
    icucfg.frequency = freq;
    icucfg.channel = PPM_ICU_CHANNEL;
    icucfg.width_cb = NULL;
    icucfg.period_cb = ppm_measurement_cb;
    icucfg.overflow_cb = NULL;
    icucfg.dier = 0;

    icuStart(&HAL_ICU_TIMER, &icucfg);
    icuStartCapture(&HAL_ICU_TIMER);
    icuEnableNotifications(&HAL_ICU_TIMER);
    return true;
}

uint16_t ppm_read(uint8_t channel)
{
    //return 0 if channel requested is out range
    if(channel >= num_channels) {
        return 0;
    }
    updated[channel] = false;
    return ppm_buffer[channel];
}

uint8_t ppm_read_bulk(uint16_t periods[], uint8_t len)
{
    uint8_t i;
    for(i = 0; (i < num_channels) && (i < len); i++) {
        periods[i] = ppm_buffer[i];
    }
    return i;
}

bool ppm_available()
{
    uint8_t i;
    for (i = 0; i < 10; i++) {
        if (updated[i]) {
            return true;
        }
    }
    return false;
}

uint8_t ppm_num_channels()
{
    return num_channels;
}

static void ppm_measurement_cb(ICUDriver *icup)
{
    uint16_t period = icuGetPeriodX(icup);
    if (period >= 2700 || buf_ptr >= 10) {
        //This is a sync pulse let's reset buffer pointer
        num_channels = buf_ptr + 1;
        buf_ptr = 0;
    } else {
        if(period > 900) {
            updated[buf_ptr] = true;
            ppm_buffer[buf_ptr] = period;
        }
        buf_ptr++;
    }
}
#endif // HAL_USE_ICU
