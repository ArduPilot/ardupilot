#include "AP_Periph.h"

#ifdef USERHOOK_INIT
void AP_Periph_FW::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_ACCEPTTRANSFER
bool AP_Periph_FW::userhook_shouldAcceptTransfer(const CanardInstance* ins,
                                                 uint64_t* out_data_type_signature,
                                                 uint16_t data_type_id,
                                                 CanardTransferType transfer_type,
                                                 uint8_t source_node_id)
{
    switch (data_type_id) {
    // put your case here
    default:
        break;
    }
    return false;
}
#endif

#ifdef USERHOOK_TRANSFERRECEIVED
void AP_Periph_FW::userhook_onTransferReceived(CanardInstance* ins,
                                               CanardRxTransfer* transfer)
{
    switch (transfer->data_type_id) {
    // put your case here
    default:
        break;
    }
}
#endif

#ifdef USERHOOK_UPDATE
void AP_Periph_FW::userhook_Update()
{
    // put your update code here
}
#endif