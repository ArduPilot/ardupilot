#include "MsgHandler_GPS_Base.h"

class MsgHandler_GPS : public MsgHandler_GPS_Base
{
public:
    MsgHandler_GPS(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec, AP_GPS &_gps,
                   uint32_t &_ground_alt_cm, float &_rel_altitude)
        : MsgHandler_GPS_Base(_f, _dataflash,_last_timestamp_usec,
                              _gps, _ground_alt_cm, _rel_altitude),
          gps(_gps), ground_alt_cm(_ground_alt_cm), rel_altitude(_rel_altitude) { };

    void process_message(uint8_t *msg);

private:
    AP_GPS &gps;
    uint32_t &ground_alt_cm;
    float &rel_altitude;
};
