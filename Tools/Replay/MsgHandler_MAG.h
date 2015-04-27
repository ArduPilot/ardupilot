#include "MsgHandler_MAG_Base.h"

class MsgHandler_MAG : public MsgHandler_MAG_Base
{
public:
    MsgHandler_MAG(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec, Compass &_compass)
        : MsgHandler_MAG_Base(_f, _dataflash, _last_timestamp_usec,_compass) {};

    virtual void process_message(uint8_t *msg);
};
