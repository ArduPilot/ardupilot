#include "SmartAudio_test.h"
#include <AP_SmartAudio/AP_SmartAudio.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

void setup();
void loop();

SmartAudioTest testInstace;


void setup()
{
    hal.console->printf("SmartAudio test\n");
    // monkey pushing buttons

}

void loop()
{
    if (!hal.console->is_initialized()) {
        return;
    }

    hal.console->printf("--------------------\n");

    testInstace.monkey_testing();

    hal.scheduler->delay(5e3); // 5 seconds
}
// do as a monkey using the vtx
    void  SmartAudioTest::monkey_testing()
    {
        const int operation=rand() % 20;
        printf("Free mem: %d\n", AP_HAL::get_HAL().util->available_memory());

        switch (operation) {
        case SMARTAUDIO_CMD_GET_SETTINGS:
            printf("\n");
            test_get_settings_res(SMARTAUDIO_RSP_GET_SETTINGS_V1);
            test_get_settings_res(SMARTAUDIO_RSP_GET_SETTINGS_V2);
            test_get_settings_res(SMARTAUDIO_RSP_GET_SETTINGS_V21);
            break;

        case SMARTAUDIO_CMD_SET_CHANNEL:
            for(uint8_t i=0;i<40;i++){
                test_set_chan_req(i);
                test_set_chan_res(i);
            }
        break;

        case SMARTAUDIO_CMD_SET_FREQUENCY:
            printf("\n");
            for(uint8_t i=0;i<6;i++){
                for(int j=0;j<8;j++){
                test_set_freq_req(AP_VideoTX::VIDEO_CHANNELS[i][j], true);
                test_set_freq_req(AP_VideoTX::VIDEO_CHANNELS[i][j], false);
                test_set_freq_res(AP_VideoTX::VIDEO_CHANNELS[i][j], true);
                test_set_freq_res(AP_VideoTX::VIDEO_CHANNELS[i][j], false);
                }

            }
            break;

        case SMARTAUDIO_CMD_SET_POWER:
            test_get_settings_res(SMARTAUDIO_RSP_GET_SETTINGS_V1);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v1, 14);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v1, 23);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v1, 27);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v1, 29);

            test_get_settings_res(SMARTAUDIO_RSP_GET_SETTINGS_V2);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v2, 14);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v2, 23);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v2, 27);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v2, 29);

            test_get_settings_res(SMARTAUDIO_RSP_GET_SETTINGS_V21);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v21, 14);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v21, 23);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v21, 27);
            test_set_power_req(SMARTAUDIO_SPEC_PROTOCOL_v21, 29);

            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v1, 14);
            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v1, 23);
            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v1, 27);
            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v1, 29);

            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v2, 14);
            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v2, 23);
            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v2, 27);
            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v2, 29);

            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v21, 14);
            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v21, 23);
            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v21, 27);
            test_set_power_res(SMARTAUDIO_SPEC_PROTOCOL_v21, 29);
            break;
        case 17: // GET_PIT_MODE_FREQ
             test_get_pit_mode_freq_req();
             break;
        case 18:
            AP_SmartAudio::get_singleton()->get_readings(AP_VideoTX::get_singleton());
            break;
        case SMARTAUDIO_CMD_SET_MODE:
            uint8_t sm_mode=0x00;
            for(uint8_t i=0;i<4;i++){
                sm_mode=sm_mode | 1 << (rand() % 4);
                test_set_mode_req(sm_mode);
                test_set_mode_res(sm_mode);
                }
            break;
        // case 19:
        //     backend->set_io_status(AP_Vtx_SerialBackend::RqStatus::idle);
        //     backend->set_status(AP_Vtx_Backend::Status::idle);
        //     break;
         }
    }


    //unit testing of the set mode request
    bool SmartAudioTest::test_set_mode_req(uint8_t sm_mode){
          uint8_t expected_request[6]={0xAA, 0x55, 0x0B, 0x01, sm_mode, 0x00};
          uint8_t crc=crc8_dvb_s2_update(0, expected_request, 5);
          expected_request[5]=crc;
          AP_SmartAudio::get_singleton()->set_operation_mode(sm_mode);
          assert_request_last_inserted(reinterpret_cast<uint8_t*>(&expected_request), 6);
          return true;
    }

    // unit testing of the set mode response
    bool SmartAudioTest::test_set_mode_res(uint8_t sm_mode){
        uint8_t response_expected[7]={0xAA, 0x55, 0x05, 0x03, sm_mode, 0x01, 0x00};
        uint8_t crc=crc8_dvb_s2_update(0, response_expected+2, 4);
        response_expected[6]=crc;
        AP_SmartAudio::get_singleton()->parse_frame_response(response_expected);
        AP_SmartAudio::get_singleton()->get_readings(AP_VideoTX::get_singleton());

        if (sm_mode==0){
          return true;
        }

        if ((sm_mode & 1<<0) && !(sm_mode & 1 << 2)){ //PIR MODE && !POR MODE && !PIT MODE DISABLED
            if (AP_VideoTX::get_singleton()->get_options()!=1){
                AP_HAL::panic("Expected mode pir mode on");
            }
        }

        if ((sm_mode & 1<<1) && !(sm_mode & 1<<2)){ //P0R MODE && !PI R MODE && !PIT MODE DISABLED
            if (AP_VideoTX::get_singleton()->get_options()!=1
            ){
                AP_HAL::panic("Expected mode por mode on");
            }
        }

        if (sm_mode & 1<<2){  //PIt MODE && !POR MODE && !PIT MODE DISABLED
            if (AP_VideoTX::get_singleton()->get_options()!=0){
                AP_HAL::panic("Expected mode pit mode disabled");
            }
        }

        if (sm_mode & 1<<3){ //LOCKED MODE
            if (AP_VideoTX::get_singleton()->get_locking()!=0){
                AP_HAL::panic("Expected mode unlocked");
            }
        }else{
            if (AP_VideoTX::get_singleton()->get_locking()!=1){
                AP_HAL::panic("Expected mode locked");
            }
        }

        return true;
    }

    // unit testing of the get settings request
    bool SmartAudioTest::test_get_settings_req(){
        uint8_t expected_req[5]={0xAA, 0x55, 0x03, 0x00, 0x9F};
        AP_SmartAudio::get_singleton()->request_settings();
        assert_request_last_inserted(reinterpret_cast<uint8_t*>(&expected_req), 5);
        return true;
    }

    // unit testing of the get settings response
    bool SmartAudioTest::test_get_settings_res(u_int8_t version){
        uint8_t response_settings_v1[10]={0xAA, 0x55, 0x01, 0x06, 0x00, 0x00, 0x01, 0x16, 0xE9, 0x4D};
        uint8_t response_settings_v2[10]={0xAA, 0x55, 0x09, 0x06, 0x01, 0x00, 0x1A, 0x16, 0xE9, 0x0A};
        uint8_t response_settings_v3[16]={0xAA, 0x55, 0x11, 0x0C, 0x00, 0x00, 0x00, 0x16, 0xE9, 0x0E, 0x03, 0x00, 0x0E, 0x14, 0x1A, 0x01};

        if (version==SMARTAUDIO_SPEC_PROTOCOL_v1){
         AP_SmartAudio::get_singleton()->parse_frame_response(response_settings_v1);
            AP_SmartAudio::get_singleton()->get_readings(AP_VideoTX::get_singleton());
            // asserts
            if (AP_VideoTX::get_singleton()->get_band()!=0){
                AP_HAL::panic("Band is not correct with response package");
            }
            if (AP_VideoTX::get_singleton()->get_channel()!=0){
                AP_HAL::panic("Channel is not correct with response package ");
            }
            if (AP_VideoTX::get_singleton()->get_frequency_mhz()!=5865){
                AP_HAL::panic("Frequency is not correct with response package  %d vs %d", 5865, AP_VideoTX::get_singleton()->get_frequency_mhz());
            }
            if (AP_VideoTX::get_singleton()->get_locking()!=1){
                AP_HAL::panic("Locking is not correct with response package");
            }
            if (AP_VideoTX::get_singleton()->get_power_mw()!=25){
                AP_HAL::panic("Power is not correct with response package");
            }
            if (AP_VideoTX::get_singleton()->get_options()!=1){
                AP_HAL::panic("Options is not correct with response package");
            }
        }
        if (version==SMARTAUDIO_SPEC_PROTOCOL_v2){

            AP_SmartAudio::get_singleton()->parse_frame_response(response_settings_v2);
            AP_SmartAudio::get_singleton()->get_readings(AP_VideoTX::get_singleton());

             // asserts
            if (AP_VideoTX::get_singleton()->get_band()!=0){
                AP_HAL::panic("Band is not correct with response package");
            }
            if (AP_VideoTX::get_singleton()->get_channel()!=1){
                AP_HAL::panic("Channel is not correct with response package ");
            }
            // is setted by channel not by freq, so, must be 5845 not 5865 wich is defined in the packet
            if (AP_VideoTX::get_singleton()->get_frequency_mhz()!=5845){
                AP_HAL::panic("Frequency is not correct with response package  %d vs %d", 5865, AP_VideoTX::get_singleton()->get_frequency_mhz());
            }
            if (AP_VideoTX::get_singleton()->get_locking()!=0){
                AP_HAL::panic("Locking is not correct with response package");
            }
            if (AP_VideoTX::get_singleton()->get_power_mw()!=25){
                AP_HAL::panic("Power is not correct with response package");
            }
            if (AP_VideoTX::get_singleton()->get_options()!=0){
                 AP_HAL::panic("Options is not correct with response package");
             }
        }
        if (version==SMARTAUDIO_SPEC_PROTOCOL_v21){

            AP_SmartAudio::get_singleton()->parse_frame_response(response_settings_v3);
            AP_SmartAudio::get_singleton()->get_readings(AP_VideoTX::get_singleton());

               // asserts
            if (AP_VideoTX::get_singleton()->get_band()!=0){
                AP_HAL::panic("Band is not correct with response package");
            }
            if (AP_VideoTX::get_singleton()->get_channel()!=0){
                AP_HAL::panic("Channel is not correct with response package ");
            }
            if (AP_VideoTX::get_singleton()->get_frequency_mhz()!=5865){
                AP_HAL::panic("Frequency is not correct with response package  %d vs %d", 5865, AP_VideoTX::get_singleton()->get_frequency_mhz());
            }
            if (AP_VideoTX::get_singleton()->get_locking()!=1){
                AP_HAL::panic("Locking is not correct with response package");
            }
            if (AP_VideoTX::get_singleton()->get_power_mw()!=25){
                AP_HAL::panic("Power is not correct with response package");
            }
            if (AP_VideoTX::get_singleton()->get_options()!=3){
                 AP_HAL::panic("Options is not correct with response package");
            }
        }

        return true;
    }
    // unit testing of the set frequency request
    bool SmartAudioTest::test_set_freq_req(uint16_t const freq, bool ispitMode){
        uint16_t pf=  applyBigEndian16(freq | (ispitMode ? SMARTAUDIO_SET_PITMODE_FREQ : 0x00));
        uint8_t expected_req_set_freq[7]={0xAA, 0x55, SMARTAUDIO_CMD_SET_FREQUENCY, 0x02, (uint8_t)pf, (uint8_t)(pf >> 8), 0xDC};

        uint8_t crc=crc8_dvb_s2_update(0, expected_req_set_freq, 6);
        expected_req_set_freq[6]=crc;
        AP_SmartAudio::get_singleton()->set_frequency(freq, ispitMode);
        assert_request_last_inserted(reinterpret_cast<uint8_t*>(&expected_req_set_freq), 7);
        return true;
    }
    // unit testing of the se frequency response.
    bool SmartAudioTest::test_set_freq_res(uint16_t const freq, bool ispitMode){
        uint16_t pf=  applyBigEndian16(freq);
        uint8_t expected_response[8]={0xAA, 0x55, 0x04, 0x04, (uint8_t)pf, (uint8_t)(pf >> 8), 0x01, 0x00};
        uint8_t crc=crc8_dvb_s2_update(0, expected_response+2, 5);
        expected_response[7]=crc;
        AP_SmartAudio::get_singleton()->parse_frame_response(expected_response);
        AP_SmartAudio::get_singleton()->get_readings(AP_VideoTX::get_singleton());

        if (freq!=AP_VideoTX::get_singleton()->get_frequency_mhz()){
             AP_HAL::panic("Response MISTMACH in channel");
        }
        return true;
    }

    // unit testing of the set channel request
    bool SmartAudioTest::test_set_chan_req(uint8_t chan_idx){
        uint8_t expected_req[6]={0xAA, 0x55, SMARTAUDIO_CMD_SET_CHANNEL, 0x01, chan_idx, 0xFF};
        uint8_t crc=crc8_dvb_s2_update(0, expected_req, 5);
        expected_req[5]=crc;
        AP_SmartAudio::get_singleton()->set_channel(chan_idx);
        assert_request_last_inserted(expected_req, 6);
        return true;
    }

       // unit testing of the set channel response
    bool SmartAudioTest::test_set_chan_res(uint8_t chan_idx){
         uint8_t expected_res[7]={0xAA, 0x55, SMARTAUDIO_RSP_SET_CHANNEL, 0x03, chan_idx, 0X01, 0xFF};
         uint8_t crc=crc8_dvb_s2_update(0, expected_res+2, 4);
         expected_res[6]=crc;
         AP_SmartAudio::get_singleton()->parse_frame_response(expected_res);
         AP_SmartAudio::get_singleton()->get_readings(AP_VideoTX::get_singleton());

         if (AP_VideoTX::get_singleton()->get_channel()!=chan_idx%8 && AP_VideoTX::get_singleton()->get_band()){
            AP_HAL::panic(" Channel parsed response is not equals than expected");
         }
         return true;
    }

    // unit testing the get pit mode frequency request
    bool SmartAudioTest::test_get_pit_mode_freq_req(){
        uint8_t expected_req[7]={0xAA, 0x55, 0x09, 0x02, 0x00, 0x40, 0xE1};
        AP_SmartAudio::get_singleton()->request_pit_mode_frequency();
        assert_request_last_inserted(expected_req, 7);
        return true;
    }

    // unit testing the set power request
    bool SmartAudioTest::test_set_power_req(uint8_t sm_version, uint8_t power){
        uint8_t expected_req[6]={0xAA, 0x55, 0x05, 0x01, AP_SmartAudio::_get_power_level_from_dbm(sm_version, power), 0x00};
        uint8_t crc=crc8_dvb_s2_update(0, expected_req, 5);
        expected_req[5]=crc;

        get_and_update_version_current_state(sm_version);

        AP_SmartAudio::get_singleton()->set_power_dbm(power);
        assert_request_last_inserted(expected_req, 6);
        return true;
    }

    // unit testing the set power response, power in dbm
    bool SmartAudioTest::test_set_power_res(uint8_t sm_version, uint8_t power){
        uint8_t response_set_power[7]={0xAA, 0x55, 0x02, 0x03, sm_version!=SMARTAUDIO_SPEC_PROTOCOL_v21?AP_SmartAudio::_get_power_level_from_dbm(sm_version, power):power, 0x01, 0xFF};

         uint8_t crc=crc8_dvb_s2_update(0, response_set_power+2, 4);
         response_set_power[6]=crc;
         get_and_update_version_current_state(sm_version);
         AP_SmartAudio::get_singleton()->parse_frame_response(response_set_power);

         AP_SmartAudio::get_singleton()->get_readings(AP_VideoTX::get_singleton());
         printf("%d", uint16_t(roundf(powf(10, power / 10.0f))));
         if (AP_VideoTX::get_singleton()->get_power_mw()-uint16_t(roundf(powf(10, power / 10.0f)))>10){
            AP_HAL::panic("Response MISTMACH in power");
         }
        return true;
    }

    // utility method to update the current state version
    void SmartAudioTest::get_and_update_version_current_state(u_int8_t version){
          // take the first register from the output buffer
        smartaudioSettings_t _current_state;

        // peek from buffer
        AP_SmartAudio::get_singleton()->vtx_states_queue.peek(&_current_state, 1);
        _current_state.version=version;
        AP_SmartAudio::get_singleton()->vtx_states_queue.update(_current_state);
    }

    // utility method to assert the equality of the last inserted request at buffer
    bool SmartAudioTest::assert_request_last_inserted(uint8_t *expectedRequest, uint8_t const size){
        AP_SmartAudio::Packet commands[AP_SmartAudio::get_singleton()->requests_queue.available()];
        AP_SmartAudio::get_singleton()->requests_queue.peek(commands, AP_SmartAudio::get_singleton()->requests_queue.available());

        for(uint8_t i=0;i<size;i++){
             if (reinterpret_cast<uint8_t*>(&commands[AP_SmartAudio::get_singleton()->requests_queue.available()-1].frame.u8RequestFrame)[i]!=expectedRequest[i]){
                 printf("\nREQUEST_QUEUE AVAIL():%d   SPACE():%d\n", AP_SmartAudio::get_singleton()->requests_queue.available(), AP_SmartAudio::get_singleton()->requests_queue.space());
                 printf("Result: ");
                 AP_SmartAudio::get_singleton()->_print_bytes_to_hex_string(reinterpret_cast<uint8_t*>(&commands[AP_SmartAudio::get_singleton()->requests_queue.available()-1].frame.u8RequestFrame), size);
                 printf("Expected: ");
                 AP_SmartAudio::get_singleton()->_print_bytes_to_hex_string(expectedRequest, size);
                 AP_HAL::panic("Request settings is different from expected at %d result: %02X  expected: %02X", i, reinterpret_cast<uint8_t*>(&commands[AP_SmartAudio::get_singleton()->requests_queue.available()-1].frame.u8RequestFrame)[i], expectedRequest[i]);
                 return false;
             }
        }
        return true;
    }


AP_HAL_MAIN();