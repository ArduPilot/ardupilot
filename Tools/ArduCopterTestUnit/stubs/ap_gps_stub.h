#ifndef AP_GPS_STUB_H_
#define AP_GPS_STUB_H_

#include "util.h"

class AP_GPS
{
	Vector3f vel;
	Location loc;

public:
    /// GPS status codes
    enum GPS_Status {
        NO_GPS = 0,             ///< No GPS connected/detected
        NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK = 5,  ///< Receiving valid messages and 3D lock, with relative-positioning improvements
    };
    GPS_Status status() {return NO_GPS;}
    GPS_Status status(uint8_t instance) {return NO_GPS;}
    uint32_t last_fix_time_ms() {return 33;}
    uint32_t last_fix_time_ms(uint8_t instance) {return 33;}
    const Vector3f &velocity() const {return vel;}
    Location location(){return loc;}
    Location location(uint8_t instance){return loc;}
    uint16_t get_hdop(uint8_t instance) const {return 2;}
    float ground_speed() const {return 2;}
    float ground_speed(uint8_t instance) const {return 1;}
    int32_t ground_course_cd(uint8_t instance) const {return 0.00005;}
    uint8_t num_sats(uint8_t instance) const {return 1;}
    uint64_t time_epoch_usec(void) {return 100000000;}
    uint64_t time_epoch_usec(uint8_t instance) {return 100000000;}

};//AP_GPS

#endif /* AP_GPS_STUB_H_ */
