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

#include "SoftSigReaderInt.h"

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

static xQueueHandle gpio_evt_queue = NULL;

void IRAM_ATTR SoftSigReaderInt::_irq_handler(void *arg)
//static void IRAM_ATTR gpio_isr_handler(void* arg)
{

    // don't printf from an interrupt....


    uint32_t gpio_num = (uint32_t) arg;
    //gpio_num_t gpio_num = (gpio_num_t) arg; // some number between 0 and 39 inclusive.

    static pulse_t pulse; // doesn't get zero'd
    static int last_transitioned_from; // 0 or 1 is the pin state state we last saw
    static uint16_t last_transitioned_time; // timestamp of the last transition

    int current_state = gpio_get_level((gpio_num_t)gpio_num);

    //printf("%d\n",current_state);

    if ( last_transitioned_from != current_state ) { // if pin state has changed

        uint32_t now = AP_HAL::micros();

        // rising edge begins event, and clear any previous pulse end measurement
        if (last_transitioned_from == 0 ) {
            pulse.w0 = now;// - last_transitioned_time;
            pulse.w1 = 0;
        }
        // falling edge is end of measurement
        if ((last_transitioned_from == 1) and ( pulse.w1 == 0 )) {
            pulse.w1 = now;// - last_transitioned_time;
        }
        last_transitioned_from = current_state;
        last_transitioned_time = now;

        // if we have both edges, push the two pulse "widths" to the signal handler..
        if ((pulse.w0 != 0) and ( pulse.w1 != 0 )) {
            _instance->sigbuf.push(pulse);
            pulse.w0 = 0;
            pulse.w1 = 0;
        }
    }

    // reset on too-big-a-big gap between pulse edges
    if ( AP_HAL::micros() - last_transitioned_time > 1000000 ) { // thats 1 second with no data at all.
        //we have probably missed some pulses
        //try to reset RCProtocol parser by returning invalid value (i.e. 0 width pulse)
        //pulse.w0 = 0;
        //pulse.w1 = 0;
        //_instance->sigbuf.push(pulse);
    }
}

// singleton instance
SoftSigReaderInt *SoftSigReaderInt::_instance;

SoftSigReaderInt::SoftSigReaderInt()
{
    _instance = this;
    printf("SoftSigReaderInt-constructed\n");
}

SoftSigReaderInt::~SoftSigReaderInt()
{

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);

}

void SoftSigReaderInt::init()
{

    printf("SoftSigReaderInt::init\n");

    // lets start with GPIO4:  input, pulled up, interrupt from rising edge and falling edge

    gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    //enable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t)1;
    // apply settings to this gpio
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, _irq_handler, (void*) GPIO_INPUT_IO_0);


}

bool SoftSigReaderInt::read(uint32_t &widths0, uint32_t &widths1)
{
    //printf("SoftSigReaderInt::read\n");

    if (sigbuf.available() >= 2) {
        pulse_t pulse;
        if (sigbuf.pop(pulse)) {
            widths0 = uint16_t(pulse.w0 - last_value);
            widths1 = uint16_t(pulse.w1 - pulse.w0);
            last_value = pulse.w1;
            return true;
        }
    }
    return false;
}

#endif // HAL_USE_EICU

#endif //CONFIG_HAL_BOARD == HAL_BOARD_ESP32
