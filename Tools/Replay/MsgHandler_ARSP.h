#include "MsgHandler.h"

class MsgHandler_ARSP : public MsgHandler
{
public:
    MsgHandler_ARSP(log_Format &_f, DataFlash_Class &_dataflash,
		    uint64_t &_last_timestamp_usec, AP_Airspeed &_airspeed) :
	MsgHandler(_f, _dataflash, _last_timestamp_usec), airspeed(_airspeed) { };

    virtual void process_message(uint8_t *msg);

private:
    AP_Airspeed &airspeed;
};
