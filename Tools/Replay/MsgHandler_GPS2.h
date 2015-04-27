#include "MsgHandler_GPS_Base.h"

// it would be nice to use the same parser for both GPS message types
// (and other packets, too...).  I*think* the contructor can simply
// take e.g. &gps[1]... problems are going to arise if we don't
// actually have that many gps' compiled in!
class MsgHandler_GPS2 : public MsgHandler_GPS_Base
{
public:
    MsgHandler_GPS2(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec, AP_GPS &_gps,
                    uint32_t &_ground_alt_cm, float &_rel_altitude)
        : MsgHandler_GPS_Base(_f, _dataflash, _last_timestamp_usec,
                              _gps, _ground_alt_cm,
                              _rel_altitude), gps(_gps),
          ground_alt_cm(_ground_alt_cm), rel_altitude(_rel_altitude) { };
    virtual void process_message(uint8_t *msg);
private:
    AP_GPS &gps;
    uint32_t &ground_alt_cm;
    float &rel_altitude;
};
