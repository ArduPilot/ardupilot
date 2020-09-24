#include "AP_SmartAudioBackend.h"
#include "smartaudio_protocol.h"
#include <AP_Math/crc.h>

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

const char *TAG="VTX-SMARTAUDIO ";
/**
 * Constructor with serial port initialization, if fails set status as "Not Connected", otherwise "Idle"
 * */
AP_SmartAudioBackend::AP_SmartAudioBackend(
    AP_Vtx_SerialBackend::Vtx_Serial_State &_state,
    AP_Vtx_Params &_params,
    uint8_t serial_instance) :
    AP_Vtx_SerialBackend(_state,_params,serial_instance,AP_SerialManager::SerialProtocol_SmartAudio)
{
    if (uart!=nullptr) {
        set_status(Status::idle);
    } else {
        set_status(Status::not_connected);
    }
}

/**
 * Sends an SmartAudio Command to the vtx, waits response on the update event
 * @param frameBuffer frameBuffer to send over the wire
 * @param size  size of the framebuffer wich needs to be sended
 */
void AP_SmartAudioBackend::send_request(smartaudioFrame_t requestFrame,uint8_t size)
{

    if (size<=0) {
        AP::logger().Write_MessageF("%s HW: %s",TAG,"CANNOT SEND REQUEST, REQUEST IS EMPTY");
        printf("ERROR - %s HW: %s",TAG,"CANNOT SEND REQUEST, REQUEST IS EMPTY");
    }
    if (uart==nullptr) {
        AP::logger().Write_MessageF("%s HW: %s",TAG,"CANNOT SEND REQUEST, UART NOT CONNECTED");
        printf("ERROR - %s HW: %s",TAG,"CANNOT SEND REQUEST, UART NOT CONNECTED");
        return;
    }
    if (rq_status!=RqStatus::idle) {
        AP::logger().Write_MessageF("%s HW: %s",TAG,"CANNOT SEND REQUEST, WIRE IS NOT IDLE");
        printf("ERROR - %s HW: %s",TAG,"CANNOT SEND REQUEST, UART IS WORKING");
        return;
    }

    request=reinterpret_cast<uint8_t*>(&requestFrame.u8RequestFrame);
    set_status(Status::processing);
    set_io_status(RqStatus::requesting);
    // pull line low
    uart->write((uint8_t)0);
    // write request
    for (int i = 0; i < size; ++i) {
        uart->write(request[i]);
    }
    set_write_time(AP_HAL::millis());
    set_io_status(RqStatus::waiting_response);


    // some VTX need this
    /*if (use_trailing_zero) {
        uart->write((uint8_t)0);
    }*/
    // Update io_time to be checked on each iteration of the main loop
    //last_io_time = AP_HAL::millis();
    //gcs().send_text(MAV_SEVERITY_WARNING, "");
#ifdef VTX_SMARTAUDIO_DEBUG
    AP::logger().Write_MessageF("%s HW: %s",TAG,"Command sent");
#endif
    print_bytes_to_hex_string(request,size);
}

/**
 * Reads the response from vtx in the wire
 **/
void AP_SmartAudioBackend::read_response()
{
    set_status(Status::processing);
    int16_t nbytes = uart->available();
    uint8_t headerSize= sizeof(smartaudioFrameHeader_t);

    if (rq_status==RqStatus::waiting_response) {
        response=new uint8_t[rx_bufsize()];
    }

    if (nbytes < 1) {
        AP::logger().Write_MessageF("%s HW: %s",TAG,"EMPTY WIRE");
        printf(" WARNING - %s HW: %s",TAG,"EMPTY WIRE");
        set_status(Status::idle);
        return;
    }
    set_io_status(RqStatus::reading_response);

    for (int i = 0; i < nbytes; ++i) {
        uint8_t b = uart->read();

        if ((responseBufferLen == 0 && b != SMARTAUDIO_SYNC_BYTE)
            || (responseBufferLen == 1 && b != SMARTAUDIO_HEADER_BYTE)) {
            responseBufferLen = 0;
        } else if (responseBufferLen < rx_bufsize()) {
            response[responseBufferLen++] = b;
        }
    }

    //last_io_time = AP_HAL::millis();

    if (responseBufferLen < headerSize) {
        set_io_status(RqStatus::idle);
        set_status(Status::idle);
        return;
    }

    uint8_t payload_len = response[3];

    if (responseBufferLen < payload_len + headerSize) {
        set_io_status(RqStatus::reading_response);
        set_status(Status::processing);
        // Not all response bytes received yet
        return;
    } else {
        responseBufferLen = payload_len + headerSize;
    }
    set_io_status(RqStatus::idle);

    uint8_t crc = response[responseBufferLen-1];
    if (crc != crc8_dvb_s2(*response, responseBufferLen-1)) {
        // crc missmatch
        AP::logger().Write_MessageF("%s HW: %s",TAG,"BAD CRC CHECK IN RESPONSE");
        printf(" ERROR - %s HW: %s",TAG,"BAD CRC CHECK IN RESPONSE");

    } else {
        parse_frame_response(nullptr);
    }
    set_status(Status::idle);
    response=nullptr;
}

/**
 * This method parse the frame response and match the current device configuration  *
 * */
void AP_SmartAudioBackend::parse_frame_response(const uint8_t *buffer)
{
    set_status(Status::processing);
    if (buffer==nullptr && response==nullptr) {
        AP::logger().Write_MessageF("%s HW: %s",TAG,"EMPTY RESPONSE");
        printf(" WARNING - %s HW: %s",TAG,"BAD CRC CHECK IN RESPONSE");
        return;
    }
    if (buffer!=nullptr) {
        smartaudioParseResponseBuffer(&_vtxSMState,buffer);
        //memcpy(&_vtxSMState,&mySettings,sizeof(mySettings));
    }
    if (response!=nullptr) {
        smartaudioParseResponseBuffer(&_vtxSMState,response);
        //memcpy(&_vtxSMState,&mySettings,sizeof(mySettings));
    }
    return;
    has_data(true);
    set_status(Status::idle);
}



void AP_SmartAudioBackend::init()
{
    set_status(Status::not_connected);
    set_io_status(RqStatus::idle);
}




