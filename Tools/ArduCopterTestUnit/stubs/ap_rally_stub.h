/*
 * ap_rally_stub.h
 *
 *  Created on: 29 mai 2014
 *      Author: valentin
 */

#ifndef AP_RALLY_STUB_H_
#define AP_RALLY_STUB_H_

class AP_Rally {
public:
    uint8_t get_rally_total() const { return 6; }
    bool set_rally_point_with_index(uint8_t i, const RallyLocation &rallyLoc) {return false;}
    bool get_rally_point_with_index(uint8_t i, RallyLocation &ret) const {return false;}

};

#endif /* AP_RALLY_STUB_H_ */
