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

/*
  SBUS output support
 */

#include "ch.h"
#include "hal.h"
#include "iofirmware.h"
#include "rc.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SBusOut/AP_SBusOut.h>

extern const AP_HAL::HAL& hal;

// usart3 is for SBUS input and output
static const SerialConfig sbus_cfg = {
    100000,   // speed
    USART_CR1_PCE | USART_CR1_M, // cr1, enable even parity
    USART_CR2_STOP_1,            // cr2, two stop bits
    0,        // cr3
    nullptr,  // irq_cb
    nullptr,  // ctx
};

// listen for parity errors on sd3 input
static event_listener_t sd3_listener;

static uint8_t sd3_config;

void sbus_out_write(uint16_t *channels, uint8_t nchannels)
{
    if (sd3_config == 0) {
        uint8_t buffer[25];
        AP_SBusOut::sbus_format_frame(channels, nchannels, buffer);
        chnWrite(&SD3, buffer, sizeof(buffer));
    }
}

// usart1 is for DSM input and (optionally) debug to FMU
static const SerialConfig dsm_cfg = {
    115200,   // speed
    0,        // cr1
    0,        // cr2
    0,        // cr3
    nullptr,  // irq_cb
    nullptr,  // ctx
};

static enum {
    RC_SEARCHING,
    RC_DSM_PORT,
    RC_SBUS_PORT
} rc_state;

/*
  init rcin on DSM USART1
 */
void AP_IOMCU_FW::rcin_serial_init(void)
{
    sdStart(&SD1, &dsm_cfg);
    sdStart(&SD3, &sbus_cfg);
    chEvtRegisterMaskWithFlags(chnGetEventSource(&SD3),
                               &sd3_listener,
                               EVENT_MASK(1),
                               SD_PARITY_ERROR);
    // disable input for SBUS with pulses, we will use the UART for
    // SBUS and FPORT
#if AP_RCPROTOCOL_SBUS_ENABLED
    AP::RC().disable_for_pulses(AP_RCProtocol::SBUS);
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED
    AP::RC().disable_for_pulses(AP_RCProtocol::SBUS_NI);
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED
    AP::RC().disable_for_pulses(AP_RCProtocol::FPORT);
#endif
}

static struct {
    uint32_t num_dsm_bytes;
    uint32_t num_sbus_bytes;
    uint32_t num_sbus_errors;
    eventflags_t sbus_error;
    uint32_t last_good_ms;
} rc_stats;

/*
  check for data on DSM RX uart
 */
void AP_IOMCU_FW::rcin_serial_update(void)
{
    uint8_t b[16];
    uint32_t n;
    uint32_t now = AP_HAL::millis();
    auto &rc = AP::RC();

    if (rc.should_search(now)) {
        rc_state = RC_SEARCHING;
    }

    // read from DSM port
    if ((rc_state == RC_SEARCHING || rc_state == RC_DSM_PORT) &&
        (n = chnReadTimeout(&SD1, b, sizeof(b), TIME_IMMEDIATE)) > 0) {
        n = MIN(n, sizeof(b));
        // don't mix two 115200 uarts
        if (sd3_config == 0) {
            rc_stats.num_dsm_bytes += n;
            for (uint8_t i=0; i<n; i++) {
                if (rc.process_byte(b[i], 115200)) {
                    rc_stats.last_good_ms = now;
                    if (!rc.should_search(now)) {
                        rc_state = RC_DSM_PORT;
                    }
                }
            }
        }
        //BLUE_TOGGLE();
    }

    // read from SBUS port
    if ((rc_state == RC_SEARCHING || rc_state == RC_SBUS_PORT) &&
        (n = chnReadTimeout(&SD3, b, sizeof(b), TIME_IMMEDIATE)) > 0) {
        eventflags_t flags;
        if (sd3_config == 0 && ((flags = chEvtGetAndClearFlags(&sd3_listener)) & SD_PARITY_ERROR)) {
            rc_stats.sbus_error = flags;
            rc_stats.num_sbus_errors++;
        } else {
            n = MIN(n, sizeof(b));
            rc_stats.num_sbus_bytes += n;
            for (uint8_t i=0; i<n; i++) {
                if (rc.process_byte(b[i], sd3_config==0?100000:115200)) {
                    rc_stats.last_good_ms = now;
                    if (!rc.should_search(now)) {
                        rc_state = RC_SBUS_PORT;
                    }
                }
            }
        }
    }
    /*
      when not using SBUS output we switch UART3 between 100000 baud
      and 115200 baud in order to support RC input protocols that are
      115200 inverted (such as FPort). If SBUS output is enabled then
      we need to disable this as the uart is shared between input and
      output
     */
    const bool sbus_out_enabled = (reg_setup.features & P_SETUP_FEATURES_SBUS1_OUT) != 0;
    if (rc_state == RC_SEARCHING &&
        now - rc_stats.last_good_ms > 2000 && (sd3_config==1 || !sbus_out_enabled)) {
        rc_stats.last_good_ms = now;
        sd3_config ^= 1;
        sdStop(&SD3);
        sdStart(&SD3, sd3_config==0?&sbus_cfg:&dsm_cfg);
    }
}

/*
  sleep for 1ms using a busy loop
 */
static void delay_one_ms(uint32_t &now)
{
    while (now == AP_HAL::millis()) ;
    now = AP_HAL::millis();
}

/*
  perform a DSM bind operation
 */
void AP_IOMCU_FW::dsm_bind_step(void)
{
    uint32_t now = last_ms;
    switch (dsm_bind_state) {
    case 1:
        palSetLineMode(HAL_GPIO_PIN_SPEKTRUM_PWR_EN, PAL_MODE_OUTPUT_PUSHPULL);
        SPEKTRUM_POWER(0);
        palSetLineMode(HAL_GPIO_PIN_SPEKTRUM_OUT, PAL_MODE_OUTPUT_PUSHPULL);
        SPEKTRUM_SET(1);
        dsm_bind_state = 2;
        last_dsm_bind_ms = now;
        break;

    case 2:
        if (now - last_dsm_bind_ms >= 500) {
            SPEKTRUM_POWER(1);
            dsm_bind_state = 3;
            last_dsm_bind_ms = now;
        }
        break;

    case 3: {
        if (now - last_dsm_bind_ms >= 72) {
            // 9 pulses works with all satellite receivers, and supports the highest
            // available protocol
            delay_one_ms(now);
            const uint8_t num_pulses = 9;
            for (uint8_t i=0; i<num_pulses; i++) {
                // the delay should be 120us, but we are running our
                // clock at 1kHz, and 1ms works fine
                delay_one_ms(now);
                SPEKTRUM_SET(0);
                delay_one_ms(now);
                SPEKTRUM_SET(1);
            }
            last_dsm_bind_ms = now;
            dsm_bind_state = 4;
        }
        break;
    }

    case 4:
        if (now - last_dsm_bind_ms >= 50) {
            // set back as input pin
            palSetLineMode(HAL_GPIO_PIN_SPEKTRUM_OUT, PAL_MODE_INPUT);
            dsm_bind_state = 0;
        }
        break;

    default:
        dsm_bind_state = 0;
        break;
    }
}
