/*
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
 *
 * Code by @davidbuzz and Claude
 */
#pragma once

#include <AP_HAL/RCOutput.h>

#ifdef __ZEPHYR__
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#endif

/* DShot support exists when the board's DTS instantiates a FlexIO DShot node. */
/* Nested #if deliberately: the DT macros must never appear in a
   preprocessor expression unless the defines guaranteeing they exist are
   already true - #if does NOT short-circuit before macro expansion. */
#if defined(__ZEPHYR__) && defined(CONFIG_NXP_FLEXIO_DSHOT)
#if DT_NODE_HAS_STATUS(DT_NODELABEL(dshot), okay) && \
    DT_NODE_EXISTS(DT_NODELABEL(ap_rcout_mux))
#define AP_ZEPHYR_DSHOT_ENABLED 1
#endif
#endif
#ifndef AP_ZEPHYR_DSHOT_ENABLED
#define AP_ZEPHYR_DSHOT_ENABLED 0
#endif

namespace Zephyr {

class RCOutput : public AP_HAL::RCOutput {
public:
    void init() override;

    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t chan) override;

    void enable_ch(uint8_t chan) override;
    void disable_ch(uint8_t chan) override;

    void write(uint8_t chan, uint16_t period_us) override;

    uint16_t read(uint8_t chan) override;
    void read(uint16_t *period_us, uint8_t len) override;

    void cork() override;
    void push() override;

#if AP_ZEPHYR_DSHOT_ENABLED
    /* Runtime protocol selection, ChibiOS parity. */
    void set_output_mode(uint32_t mask, enum output_mode mode) override;
    enum output_mode get_output_mode(uint32_t &mask) override;

    /* Called at ~1 kHz from Scheduler::_rcout_thread_fn. */
    void dshot_tick();
#endif

private:
    static constexpr uint8_t NUM_CHANNELS = 14;

#ifdef __ZEPHYR__
    struct PWMChannelMap {
        const struct device *dev;
        uint8_t hw_channel;
    };

    PWMChannelMap _map[NUM_CHANNELS] = {};
    bool _map_ready = false;
#endif

    bool _enabled[NUM_CHANNELS] = {};
    bool _dev_ready[NUM_CHANNELS] = {};
    uint16_t _period_us[NUM_CHANNELS] = {};
    uint16_t _freq_hz[NUM_CHANNELS] = {};

    bool _corked = false;
    bool _dirty[NUM_CHANNELS] = {};

#if AP_ZEPHYR_DSHOT_ENABLED
    /* Nonzero while FlexIO owns the FMU_CH1-8 pad bank; bit n = AP
       channel n speaks DShot. Statics live in .bss - no initialisers. */
    uint32_t _dshot_mask;
    enum output_mode _dshot_mode;
#endif

#ifdef __ZEPHYR__
    void _apply_channel(uint8_t chan);
#endif
};

}  // namespace Zephyr
