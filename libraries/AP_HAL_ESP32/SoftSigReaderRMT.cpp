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
 * Code by David "Buzz" Bussenschutt and others
 *
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "SoftSigReaderRMT.h"

using namespace ESP32;

#define RMT_CLK_DIV      10    /*!< RMT counter clock divider */
#define RMT_TICK_US    (80000000/RMT_CLK_DIV/1000000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */
#define PPM_IMEOUT_US  3500   /*!< RMT receiver timeout value(us) */


// the RMT peripheral on the esp32 to do this ? looks plausible.
// https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/rmt.html
// with an example here for both transmit and receive of IR signals:
// https://github.com/espressif/esp-idf/tree/2f8b6cfc7/examples/peripherals/rmt_nec_tx_rx

extern const AP_HAL::HAL& hal;


void SoftSigReaderRMT::init()
{

    printf("SoftSigReaderRMT::init\n");

    printf("%s\n",__PRETTY_FUNCTION__);

    // in case the peripheral was left in a bad state, such as reporting full buffers, this can help clear it, and can be called repeatedly if need be.
    //periph_module_reset(PERIPH_RMT_MODULE);


    rmt_config_t config;
    config.rmt_mode = RMT_MODE_RX;
    config.channel = RMT_CHANNEL_0;

#ifndef HAL_ESP32_RMT_RX_PIN_NUMBER
    #error HAL_ESP32_RMT_RX_PIN_NUMBER undefined in libraries/AP_HAL_ESP32/boards/esp32... .h
#endif

    config.gpio_num = (gpio_num_t)HAL_ESP32_RMT_RX_PIN_NUMBER;

    config.clk_div = RMT_CLK_DIV;
    config.mem_block_num = 1;
    config.rx_config.filter_en = true;
    config.rx_config.filter_ticks_thresh = 100;
    config.rx_config.idle_threshold = PPM_IMEOUT_US * (RMT_TICK_US);

    rmt_config(&config);
    rmt_driver_install(config.channel, 1000, 0);
    rmt_get_ringbuf_handle(config.channel, &rb);

    // we could start it here, but then we get RMT RX BUFFER FULL message will we start calling read()
    //rmt_rx_start(config.channel, true);
}


/*
this is what the inside of a rmt_item32_t looks like:
// noting that its 15bits+1bit and another 15bits+ 1bit, so values over 32767 in "duration" bits don't work.
// and the entire size is 32bits
typedef struct rmt_item32_s {
    union {
        struct {
            uint32_t duration0 :15;
            uint32_t level0 :1;
            uint32_t duration1 :15;
            uint32_t level1 :1;
        };
        uint32_t val;
    };
} rmt_item32_t;
*/

bool SoftSigReaderRMT::read(uint32_t &widths0, uint32_t &widths1)
{

    //printf("%s\n",__PRETTY_FUNCTION__);

    // delayed start till the threads are initialised and we are ready to read() from it....
    if (! started )  {
        rmt_rx_start(RMT_CHANNEL_0, true);
        started = true;
    }

    size_t rx_size = 0;

    static uint32_t channeldata0[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; // don't hardcode this
    static uint32_t channeldata1[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; // don't hardcode this
    static int channelpointer = -1;

    int channels;

    // always give priority to handling the RMT queue first
    rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 0);

    if (item) {

        channels = (rx_size / 4) - 1;
        //printf("PPM RX %d (%d) channels: ", channels, rx_size);
        for (int i = 0; i < channels; i++) {
            //printf("%04d ", ((item+i)->duration1 + (item+i)->duration0) / RMT_TICK_US);
            channeldata0[i] = ((item+i)->duration0)/RMT_TICK_US;
            channeldata1[i] = ((item+i)->duration1)/RMT_TICK_US;
            if ( channelpointer < 0 ) {
                channelpointer = 0;
            }
        }
        //printf("\n");

        vRingbufferReturnItem(rb, (void*) item);
        item = nullptr;
    }

    // each time we are externally called as read() we'll give some return data to the caller.
    if ( channelpointer >= 0 ) {
        widths0 = uint16_t(channeldata0[channelpointer]);
        widths1 = uint16_t(channeldata1[channelpointer]);

        //printf("  hi low ch -> %d %d %d\n",width_high,width_low,channelpointer);
        channelpointer++;

        // in here, after the 8th channel, we're going to re-insert the "wide" pulse that is the idle pulse for ppmsum:
        if ( channelpointer == 9 ) {
            widths0 = 3000; // must together add up over 2700
            widths1 =  1000;
        }
        if ( channelpointer > 9 ) {
            channelpointer = 0;
            return false;
        }
        return true;
    }

    return false;

}


#endif //CONFIG_HAL_BOARD == HAL_BOARD_ESP32
