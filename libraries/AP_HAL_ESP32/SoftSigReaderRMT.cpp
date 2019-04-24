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

#include "SoftSigReaderRMT.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

// future possible TODO - can we use the RMT peripheral on the esp32 to do this ? looks plausible. 
// https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/rmt.html
// with an example here for both transmit and recieve of IR signals:
// https://github.com/espressif/esp-idf/tree/2f8b6cfc7/examples/peripherals/rmt_nec_tx_rx


//for now, we use GPIO interrupts,refer here:
// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/gpio/main/gpio_example_main.c


#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

using namespace ESP32;
extern const AP_HAL::HAL& hal;

#if HAL_USE_EICU == TRUE

#define GPIO_INPUT_IO_0     (gpio_num_t)4
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))
#define ESP_INTR_FLAG_DEFAULT 0


// singleton instance
SoftSigReaderRMT *SoftSigReaderRMT::_instance;

SoftSigReaderRMT::SoftSigReaderRMT()
{
    _instance = this;
    printf("SoftSigReaderRMT-constructed\n");
}

SoftSigReaderRMT::~SoftSigReaderRMT()
{
    
    //remove isr handler for gpio number.
    //gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    
    rmt_rx_stop((rmt_channel_t)0 );

    //vTaskDelete(NULL);

}

void SoftSigReaderRMT::init()
{

	printf("SoftSigReaderRMT::init\n");

	//---------------------------------------
	#define RMT_RX_CHANNEL    (rmt_channel_t)0     
	/*!< RMT channel for receiver */ 
	#define RMT_CLK_DIV      100   
	 /*!< RMT counter clock divider */
	#define rmt_item32_tIMEOUT_US  9500   
	/*!< RMT receiver timeout value(us) */
	#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)  
	 /*!< RMT counter value for 10 us.(Source clock is APB clock) */
	
	
    rmt_config_t rmt_rx;
    rmt_rx.channel = (rmt_channel_t)RMT_RX_CHANNEL;
    rmt_rx.gpio_num = GPIO_INPUT_IO_0; /*!< GPIO number for receiver */
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0); // chan_num, buffer_size, flags


	// do I need to sleep here at all ? 
	
    rmt_channel_t channel = RMT_RX_CHANNEL;
    
    
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);
    
}

#define RMT_RX_ACTIVE_LEVEL  1   /*!< the data is active hi */

#define NEC_HEADER_HIGH_US    9000                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4500                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_MARGIN         20                          /*!< NEC parse margin time */
#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */



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


  static bool ret = true;

  if ( ret == true )  return true;

  printf("SoftSigReaderRMT::read\n");

    //while(rb) {
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        
        static pulse_t pulse; // doesn't get zero'd
        
        // transition/s from 0->1
        if ( item->level0 == 0 && item->level1 != 0 ) { 
			pulse.w0 = item->duration0;
        } 
        // transition/s from 1->0
        if ( item->level0 == 1 && item->level1 != 1 ) { 
        	pulse.w1  =  item->duration1;
        }         
        widths0 = uint16_t(pulse.w0 - last_value);
        widths1 = uint16_t(pulse.w1 - pulse.w0);
        last_value = pulse.w1;
          
        //after parsing the data, return spaces to ringbuffer.
        vRingbufferReturnItem(rb, (void*) item);
         
    //}
  printf("SoftSigReaderRMT::read-finished\n");
  
	return true;
}

#endif // HAL_USE_EICU

#endif //CONFIG_HAL_BOARD == HAL_BOARD_ESP32
