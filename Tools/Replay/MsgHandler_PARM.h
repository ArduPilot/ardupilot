#include <MsgHandler.h>

class MsgHandler_PARM : public MsgHandler
{
public:
    MsgHandler_PARM(log_Format &_f, DataFlash_Class &_dataflash, uint64_t _last_timestamp_usec) : MsgHandler(_f, _dataflash, _last_timestamp_usec) {};

    virtual void process_message(uint8_t *msg);
};
