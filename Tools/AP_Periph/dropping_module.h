/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

// #ifdef HAL_PERIPH_ENABLE_SBUS

class DROP_Module {
public:
    DROP_Module();

    void init(AP_HAL::UARTDriver *uart);
    bool update();

    enum DROP_Module_st{
        NONE = 0,
        DROPING = 1,
        ERROR = 2
    };
    DROP_Module_st st = NONE;

    void change_state(uint8_t new_st);
private:
    AP_HAL::UARTDriver *uart;


    uint8_t len;
    uint8_t state;
    uint32_t last_read_ms;
    uint32_t error_count;

    
typedef struct
{
    // FieldTypes                    //
    uint8_t    channel_id;                    // bit len 8
    
    struct
    {     //COMMAND_LONG ( #76 )
        // len = 33 + header +
        //uint8_t[60]   data;                      // Dynamic Array 8bit[60] max items
        uint8_t target_system;//	1		System which should execute the command
        uint8_t target_component;//	1		Component which should execute the command, 0 for all components
        uint16_t command;//	2	MAV_CMD	Command ID (of command to send).
        uint8_t confirmation;//	1		0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
        float param1;   //	4		Parameter 1 (for the specific command).
        float param2;   //	4		Parameter 2 (for the specific command).
        float param3;   //	4		Parameter 3 (for the specific command).
        float param4;   //	4		Parameter 4 (for the specific command).
        float param5;   //	4		Parameter 5 (for the specific command).
        float param6;   //	4		Parameter 6 (for the specific command).
        float param7;   //	4       Parameter 7 (for the specific command).
        // 
    } buffer;

} drop_mod_data_cmd_long;

    void dropping_mod(void);
    void sw_update();
    void adc_update();
};

// #endif // HAL_PERIPH_ENABLE_SBUS
