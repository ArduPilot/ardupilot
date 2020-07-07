
#include "AP_SmartAudio.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_SmartAudio::var_info[] = {

    // @Param: PROTOCOL
    // @DisplayName: SmartAudio enabled
    // @Description:
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLED", 1, AP_SmartAudio, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TRAILZERO
    // @DisplayName: Enable trailing zero
    // @Description: If enabled the trailing zero will be sent after SA frame. Some VTX requires that.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TRAILZERO", 2, AP_SmartAudio, use_trailing_zero, 0),

    AP_GROUPEND
};



AP_SmartAudio::AP_SmartAudio()
{
    AP_Param::setup_object_defaults(this, var_info);
    singleton = this;

}

void AP_SmartAudio::init()
{
  device= new SPEC_SmartAudioDevice();
  initPort();
  //if(enabled){
    this->autoDiscoverSpec();
  //}
}

/**
 * OP: Initialization of communication with the serial interface
 */
void AP_SmartAudio::initPort(){
    port = AP_SerialManager::get_singleton()->find_serial(AP_SerialManager::SerialProtocol_SmartAudio, 0);

    if (port != nullptr) {
        //AP_SerialManager::get_singleton()->find_baudrate(AP_SerialManager::SerialProtocol_SmartAudio, 0);
        /// TODO: FIND BAUDRATE FROM SERIALMANAGER #find_baudrate
        port->begin(SMARTAUDIO_DEFAULT_BAUD, SA_BUFFER_SIZE, SA_BUFFER_SIZE);  // Set buffers to MACRO VALUE
        port->set_stop_bits(2);
        port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        port->set_options(AP_HAL::UARTDriver::OPTION_HDPLEX);
    }else{
        logToStdOut("initPort: SMART AUDIO PORT COM NOT FOUND, ARE YOU SURE THERE IS ONE OF THIS ?");
        enabled=false;
        return;
    }

    logToStdOut("initPort: SMART AUDIO PORT COM initialized");
    gcs().send_text(MAV_SEVERITY_WARNING, "SMART AUDIO PORT COM initialized");
};

/**
 * Launch an initial getSettings to autodiscover the spec version
 */
void AP_SmartAudio::autoDiscoverSpec(){

     if(device->isInitialized()){
        logToStdOut("autoDiscoverSpec: Refresh from vtx setttings requested");
    }else{
        logToStdOut("autoDiscoverSpec: Initiliazing vtx setttings requested");
    }

    if(!this->is_waiting_response()){
        this->getSettings();
    }else{
         logToStdOut("autoDiscoverSpec: AUTODISCOVERY ABOTERD vtx is being requested");
    }
};

/**
 * Public method to query vtx state
 */
void AP_SmartAudio::getSettings(){
    smartaudioFrame_t frame= smartaudioFrame_t();
    uint8_t size=smartaudioFrameGetSettings(&frame);
    send_frame(reinterpret_cast<uint8_t*>(&frame.u8RequestFrame),size);
}
/**
 * Public method to set power level in the vtx state
 * @param powerLevel Level of power to set into the vtx
 * */
 void AP_SmartAudio::setPower(uint8_t powerLevel){
    smartaudioFrame_t frame= smartaudioFrame_t();
    uint8_t size=smartaudioFrameSetPower(&frame,powerLevel);
    send_frame(reinterpret_cast<uint8_t*>(&frame.u8RequestFrame),size);
 }
/**
 * Public method to set frecuency in the vtx state
 * @param frecuency Frecuency to set into the vtx
 * @param inPitModeFrec Frecuency to set when is in pit mode
 * */
 void AP_SmartAudio::setFrecuency(uint16_t frecuency,bool isPitModeFrec){
    smartaudioFrame_t frame= smartaudioFrame_t();
    uint8_t size=smartaudioFrameSetFrequency(&frame,frecuency,isPitModeFrec);
    send_frame(reinterpret_cast<uint8_t*>(&frame.u8RequestFrame),size);
 }



/**
 * SPEC: The SmartAudio line need to be low before a frame is sent. If the host MCU can’t handle this it can be done by sending a 0x00 dummy byte in front of the actual frame.
 */
void AP_SmartAudio::beforeSend(){
    port->write((uint8_t)0);
}

/**
 * Sends an SmartAudio Command to the vtx, waits response on the update event
 * @param frameBuffer frameBuffer to send over the wire
 * @param size  size of the framebuffer wich needs to be sended
 */
void AP_SmartAudio::send_frame(uint8_t frameBuffer[],uint8_t size){
     if(this->port==nullptr){
        logToStdOut("SEND_COMMAND: NO UART CONNECTTED");
        return;
    }
    // pull line low
    beforeSend();

    // write request
    for(int i = 0; i < size; ++i) {
	    port->write(frameBuffer[i]);
    }

    // some VTX need this
    if (use_trailing_zero) {
        port->write((uint8_t)0);
    }

    // Update io_time to be checked on each iteration of the main loop
    last_io_time = AP_HAL::millis();

    gcs().send_text(MAV_SEVERITY_WARNING, "Command sent");
    print_bytes_to_hex_string(frameBuffer,size);
}


/**
 * READ RESPONSE FROM SERIAL BUFFER, THEN PARSES
 * */
void AP_SmartAudio::read_response()
{
    int16_t nbytes = port->available();

    if (nbytes < 1) {
        logToStdOut("read_response: EMPTY WIRE");
        return;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "Incoming bytes");

    for (int i = 0; i < nbytes; ++i) {
        uint8_t b = port->read();

        if ((buffer_len == 0 && b != SMARTAUDIO_SYNC_BYTE)
            || (buffer_len == 1 && b != SMARTAUDIO_HEADER_BYTE))
        {
            buffer_len = 0;
        }
        else if (buffer_len < SA_BUFFER_SIZE) {
            buffer[buffer_len++] = b;
        }
    }

    last_io_time = AP_HAL::millis();

    if (buffer_len < header_size) {
        return;
    }

    uint8_t payload_len = buffer[3];

    if (buffer_len < payload_len + header_size) {
        // Not all response bytes received yet
        return;
    }
    else {
        buffer_len = payload_len + header_size;
    }

    uint8_t crc = buffer[buffer_len-1];

    if (crc != crc_crc8_dvb_s2(buffer, buffer_len-1)) {
        // crc missmatch
        gcs().send_text(MAV_SEVERITY_WARNING, "CRC missmatch");
        return;
    }
    parseFrameResponse(buffer);
}

/**
 * This method parse the frame response and match the current device configuration  *
 * */
void AP_SmartAudio::parseFrameResponse(uint8_t frameBuffer[]){
      gcs().send_text(MAV_SEVERITY_WARNING, "parsing");
      uint8_t command=frameBuffer[2];
      smartaudioSettingsResponseFrame_s settingsResponseFrame;


    switch (command){
        case SMARTAUDIO_RSP_GET_SETTINGS_V1: // GET_SETTINGS SA v1

            memcpy(&settingsResponseFrame, frameBuffer, sizeof(settingsResponseFrame));
            device->setVersion(SA_SPEC_PROTOCOL_v1);
            parseSettings(settingsResponseFrame);
            break;

        case SMARTAUDIO_RSP_GET_SETTINGS_V2:                // GET_SETTINGS SA v2

            memcpy(&settingsResponseFrame, frameBuffer, sizeof(settingsResponseFrame));
            device->setVersion(SA_SPEC_PROTOCOL_v2);
            parseSettings(settingsResponseFrame);
            break;

        case SMARTAUDIO_RSP_GET_SETTINGS_V21:               // GET_SETTINGS SA v2.1

            memcpy(&settingsResponseFrame, frameBuffer, sizeof(settingsResponseFrame));
            device->setVersion(SA_SPEC_PROTOCOL_v21);
            parseSettings(settingsResponseFrame);
            break;

        case SMARTAUDIO_RSP_SET_CHANNEL:
            break;
        case SMARTAUDIO_RSP_SET_FREQUENCY:
            break;
        case SMARTAUDIO_RSP_SET_MODE:
            break;
        case SMARTAUDIO_RSP_SET_POWER:
            break;


        default:
            logToStdOut("INVALID COMMAND REQUESTED");
    }
}


/**
 * Pase settings returned by the special getSettings command wich difers from the rest of the operation
 * @param response frame response
 */
void AP_SmartAudio::parseSettings(smartaudioSettingsResponseFrame_s response){
    device->setChannel(response.channel);
    device->setPowerLevel(response.power);
    device->setOperationMode(response.operationMode);
    device->setCurrentFrequency(response.frequency);
}





/**
 * KNOCK, KNOCK, KNOCKING ON THE HEAVEN DOORS
 * */
void AP_SmartAudio::update()
{
    if (!enabled && port == nullptr){
        return;
    }

    if (is_waiting_response()) {
        logToStdOut("heaven: IM WAITING AN SMART AUDIO RESPONSE");
        read_response();
    }
    else {

        /*
        if (get_settings_request) {
            send_command(SA_SPEC_CMD_GET_SETTINGS, nullptr, 0);
            get_settings_request = false;
            gcs().send_text(MAV_SEVERITY_WARNING, "SA Refresh");
        }
        else if (power_level_request > -1) {
            uint8_t pwr = (uint8_t)power_level_request;
            send_command(SA_SPEC_CMD_SET_POWER, &pwr, 1);
            power_level_request = -1;
        }
        */
    }
}

AP_SmartAudio *AP_SmartAudio::singleton;