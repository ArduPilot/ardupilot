#include "MsgHandler.h"

class MsgHandler_ATT : public MsgHandler
{
public:
    MsgHandler_ATT(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec, Vector3f &_attitude)
        : MsgHandler(_f, _dataflash, _last_timestamp_usec), attitude(_attitude)
        { };
    virtual void process_message(uint8_t *msg);

private:
    Vector3f &attitude;
};
