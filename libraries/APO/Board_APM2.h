/*
 * Board_APM2.h
 *
 *  Created on: Dec 7, 2011
 *
 */

#ifndef Board_APM2_H_
#define Board_APM2_H_

#include "AP_Board.h"

namespace apo {

class Board_APM2 : public AP_Board {
public:
    Board_APM2(mode_e mode, MAV_TYPE vehicle, options_t options);
private:
};

} // namespace apo

#endif /* AP_BOARD_APM2_H_ */
// vim:ts=4:sw=4:expandtab
