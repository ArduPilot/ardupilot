#include "MsgHandler.h"

class MsgHandler_AHR2 : public MsgHandler
{
public:
    MsgHandler_AHR2(log_Format &_f, DataFlash_Class &_dataflash,
                    uint64_t &_last_timestamp_usec, Vector3f &_ahr2_attitude)
        : MsgHandler(_f, _dataflash,_last_timestamp_usec),
          ahr2_attitude(_ahr2_attitude) { };

    virtual void process_message(uint8_t *msg);

private:
    Vector3f &ahr2_attitude;
};
