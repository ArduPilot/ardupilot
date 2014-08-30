/*
 * rc_input_stub.h
 *
 *  Created on: 28 mai 2014
 *      Author: valentin
 */

#ifndef RC_INPUT_STUB_H_
#define RC_INPUT_STUB_H_


class RCInput {
public:
    uint8_t num_channels(void) {return 4;}
    uint16_t read(uint8_t ch) {return 1;}
    bool set_overrides(int16_t *overrides, uint8_t len) {return true;};
};//RCInput


#endif /* RC_INPUT_STUB_H_ */
