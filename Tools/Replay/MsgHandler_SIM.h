#include "MsgHandler.h"

class MsgHandler_SIM : public MsgHandler
{
public:
    MsgHandler_SIM(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec,
                   Vector3f &_sim_attitude)
        : MsgHandler(_f, _dataflash, _last_timestamp_usec),
          sim_attitude(_sim_attitude)
        { };

    virtual void process_message(uint8_t *msg);

private:
    Vector3f &sim_attitude;
};
