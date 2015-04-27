#include "MsgHandler.h"

class MsgHandler_NTUN_Copter : public MsgHandler
{
public:
    MsgHandler_NTUN_Copter(log_Format &_f, DataFlash_Class &_dataflash,
			   uint64_t &_last_timestamp_usec, Vector3f &_inavpos)
	: MsgHandler(_f, _dataflash, _last_timestamp_usec), inavpos(_inavpos) {};

    virtual void process_message(uint8_t *msg);

private:
    Vector3f &inavpos;
};
