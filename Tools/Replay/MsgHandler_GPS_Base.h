#include <MsgHandler.h>

#ifndef AP_MSGHANDLER_GPS_BASE_H
#define AP_MSGHANDLER_GPS_BASE_H

class MsgHandler_GPS_Base : public MsgHandler
{

public:
    MsgHandler_GPS_Base(log_Format &_f, DataFlash_Class &_dataflash,
                        uint64_t &_last_timestamp_usec, AP_GPS &_gps,
                        uint32_t &_ground_alt_cm, float &_rel_altitude)
        : MsgHandler(_f, _dataflash, _last_timestamp_usec),
          gps(_gps), ground_alt_cm(_ground_alt_cm),
          rel_altitude(_rel_altitude) { };

protected:
    void update_from_msg_gps(uint8_t imu_offset, uint8_t *data, bool responsible_for_relalt);

private:
    AP_GPS &gps;
    uint32_t &ground_alt_cm;
    float &rel_altitude;
};

#endif

