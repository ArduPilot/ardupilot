#include "MsgHandler.h"

class MsgHandler_BARO : public MsgHandler
{
public:
    MsgHandler_BARO(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec, AP_Baro &_baro)
        : MsgHandler(_f, _dataflash, _last_timestamp_usec), baro(_baro) { };

    virtual void process_message(uint8_t *msg);

private:
    AP_Baro &baro;
};
