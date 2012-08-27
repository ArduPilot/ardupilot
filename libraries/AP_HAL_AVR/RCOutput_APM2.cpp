
#include <avr/interrupt.h>

#include <AP_HAL_AVR.h>
#include "RCOutput.h"
using namespace AP_HAL_AVR;

/* No init argument required */
void APM2RCOutput::init(void* machtnicht) {

}

/* Output freq (1/period) control */
void APM2RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {

}

uint16_t APM2RCOutput::get_freq(uint8_t ch) {

}

/* Output active/highZ control, either by single channel at a time
 * or a mask of channels */
void APM2RCOutput::enable_ch(uint8_t ch) {
    enable_mask(1 << ch);
}

void APM2RCOutput::enable_mask(uint32_t chmask) {

}

void APM2RCOutput::disable_ch(uint8_t ch) {
    disable_mask(1 << ch);
}

void APM2RCOutput::disable_mask(uint32_t chmask) {

}

/* Output, either single channel or bulk array of channels */
void APM2RCOutput::write(uint8_t ch, uint16_t period_ms) {

}

void APM2RCOutput::write(uint8_t ch, uint16_t* period_ms, uint8_t len) {

}


/* Read back current output state, as either single channel or
 * array of channels. */
uint16_t APM2RCOutput::read(uint8_t ch) {

}

void APM2RCOutput::read(uint16_t* period_ms, uint8_t len) {

}

