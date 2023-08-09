/*
  custom code for specific boards
 */
#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include "support.h"

#ifdef AP_BOOTLOADER_CUSTOM_HERE4
/*
  reset here4 LEDs
*/
static void bootloader_custom_Here4(void)
{
    for (uint8_t n=0; n<10; n++) {
        const uint8_t num_leds = 4;
        const uint32_t min_bits = num_leds*25+50;
        const uint8_t num_leading_zeros = 8-min_bits%8 + 50;
        const uint32_t output_stream_byte_length = (min_bits+7)/8;
        palSetLineMode(HAL_GPIO_PIN_LED_DI, PAL_MODE_OUTPUT_PUSHPULL);
        palSetLineMode(HAL_GPIO_PIN_LED_SCK, PAL_MODE_OUTPUT_PUSHPULL);
        int l = 100;
        while (l--) {
            for (uint32_t i=0; i<output_stream_byte_length; i++) {
                for (uint8_t bit = 0; bit < 8; bit++) {
                    uint32_t out_bit_idx = i*8+bit;
                    uint8_t bit_val;
                    if (out_bit_idx < num_leading_zeros) {
                        bit_val = 0;
                    } else if ((out_bit_idx-num_leading_zeros) % 25 == 0) {
                        bit_val = 1;
                    } else {
                        bit_val = 0;
                    }

                    palClearLine(HAL_GPIO_PIN_LED_SCK);
                    palWriteLine(HAL_GPIO_PIN_LED_DI, bit_val);
                    palSetLine(HAL_GPIO_PIN_LED_SCK);
                }
            }
        }
        chThdSleepMilliseconds(10);
    }
}
#endif // AP_BOOTLOADER_CUSTOM_HERE4

void custom_startup(void)
{
#ifdef AP_BOOTLOADER_CUSTOM_HERE4
    bootloader_custom_Here4();
#endif
}
