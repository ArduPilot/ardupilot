#include "AP_Vtx_SerialBackend.h"
#include "smartaudio_protocol.h"
#include <AP_LOGGER/AP_Logger.h>


/***
 *
 * Smart Audio protocol vtx backend.
 * One day, it's must be a subclass for AP_VTX_RequestResponse_Backend superclass
 *
 * */

/***
 *      -----------             REQ:FULL  RES:EMPTY  SYNCSTATUS: SYNC|DESYNC|UNSYNC  STATUS:IDLE         RQSTATUS:IDLE
 *      REQ => VTX              REQ:FULL  RES:EMPTY  SYNCSTATUS: SYNC|DESYNC|UNSYNC  STATUS:PROCESSING   RQSTATUS:REQUESTING
 *       MEANTIME               REQ:FULL  RES:EMPTY  SYNCSTATUS: SYNC|DESYNC|UNSYNC  STATUS:PROCESSING   RQSTATUS:WAITINGRESPONSE
 *      RES <== VTX             REQ:FULL  RES:FULL   SYNCSTATUS: SYNC|DESYNC|UNSYNC  STATUS:PROCESSING   RQSTATUS:READINGRESPONSE
 *  PROCESSING-RESPONSE         REQ:EMPTY RES:FULL   SYNCSTATUS: SYNC|DESYNC|UNSYNC  STATUS:PROCESSING   RQSTATUS:IDLE
 *  PROCESSED-RESPONSE          REQ:EMPTY RES:EMPTY  SYNCSTATUS: SYNC|DESYNC|UNSYNC  STATUS:IDLE         RQSTATYS:IDLE
 *
 *
 *
 */

class AP_SmartAudioBackend: public AP_Vtx_SerialBackend
{

private:

    smartaudioSettings_t _vtxSMState;

public:
    AP_SmartAudioBackend(
        //AP_Vtx_Backend::Vtx_State &_state,
        Vtx_Serial_State &_state,
        AP_Vtx_Params &_params,
        uint8_t serial_instance);
    void init();

    void send_request(smartaudioFrame_t requestFrame,uint8_t size);
    void parse_frame_response( const uint8_t *buffer);

    uint8_t band()
    {
        return 0;   /// MUST SEARCH INTO FREQ TABLES TO DEFINE BAND??
    }
    uint8_t channel()
    {
        return _vtxSMState.channel;
    }
    uint8_t frecuency()
    {
        return _vtxSMState.frequency;
    }
    uint8_t power()
    {
        return _vtxSMState.power;
    }

    AP_Vtx_Backend::Type type()
    {
        return Type::smartaudio;
    }
    uint8_t version()
    {
        return _vtxSMState.version;
    }
    bool is_in_pit_mode_disabled()
    {
        return _vtxSMState.pitmodeDisabled;
    }
    bool is_in_range_pit_mode()
    {
        return _vtxSMState.pitmodeInRangeActive && !_vtxSMState.pitmodeOutRangeActive;
    }
    bool is_out_range_pit_mode()
    {
        return !_vtxSMState.pitmodeInRangeActive && _vtxSMState.pitmodeOutRangeActive;
    }
    bool is_unlocked()
    {
        return _vtxSMState.unlocked;
    }
    bool is_version_one()
    {
        return _vtxSMState.version==1;
    }
    bool is_version_two()
    {
        return _vtxSMState.version==2;
    }
    bool is_version_two_one()
    {
        return _vtxSMState.version==3;
    }
    void read_response();
    // flag to query if the vtx has good queality data in
    bool has_data() const
    {
        return data_is_ready && this->_status!=Status::not_connected && this->_status!=Status::desynchronized;
    }

protected:
    uint16_t rx_bufsize() const
    {
        return 16;
    }
    uint16_t tx_bufsize() const
    {
        return 16;
    }


    uint8_t *request=nullptr;
    uint8_t *response=nullptr;

    uint8_t responseBufferLen;   // Control reading response between loop cycles

    // Set Flag for data ready, maybe one day use a lock
    void has_data(bool flag)
    {
        data_is_ready=flag;
    }

    bool data_is_ready=false;



    // uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length);
};