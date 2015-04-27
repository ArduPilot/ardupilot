#include <MsgHandler.h>
#include <VehicleType.h>

class MsgHandler_MSG : public MsgHandler
{
public:
    MsgHandler_MSG(log_Format &_f, DataFlash_Class &_dataflash,
                   uint64_t &_last_timestamp_usec,
                   VehicleType::vehicle_type &_vehicle, AP_AHRS &_ahrs) :
        MsgHandler(_f, _dataflash, _last_timestamp_usec),
        vehicle(_vehicle), ahrs(_ahrs) { }


    virtual void process_message(uint8_t *msg);

private:
    VehicleType::vehicle_type &vehicle;
    AP_AHRS &ahrs;
};
