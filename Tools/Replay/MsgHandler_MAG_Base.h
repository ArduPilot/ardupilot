#ifndef MSGHANDLER_MAG_BASE_H
#define MSGHANDLER_MAG_BASE_H

#include "MsgHandler.h"

class MsgHandler_MAG_Base : public MsgHandler
{
public:
    MsgHandler_MAG_Base(log_Format &_f, DataFlash_Class &_dataflash,
                        uint64_t &_last_timestamp_usec, Compass &_compass)
	: MsgHandler(_f, _dataflash, _last_timestamp_usec), compass(_compass) { };

protected:
    void update_from_msg_compass(uint8_t compass_offset, uint8_t *msg);

private:
    Compass &compass;
};

#endif // MSGHANDLER_MAG_BASE
