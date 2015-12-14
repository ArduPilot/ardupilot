/*
 *       Example sketch to demonstrate use of AP_SBUS library.
 *       Code by Daniel Frenzel
 */


#include <AP_SBUS/AP_SBUS.h>
#include <AP_HAL_Linux/sbus.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// setup routine
static void setup() {
    // introduction
    hal.console->printf("ArduPilot SBUS test\n\n");
}

void loop() {
    uint16_t testarray[25] {0};
    uint8_t testarray2[25] {0};
    testarray[0] = 0x0f;
    testarray[24] = 0x00;
    
    testarray2[0] = 0x0f;
    testarray2[24] = 0x00;
    
    AP_SBUS decoder;
    
    int time = AP_HAL::millis();
    for(int i = 0; i < 10000000; i++) {
        decoder.process_frame(&testarray[0], 25);
        for(int j = 0; j < 18; j++) {
            int channel = decoder.get_channel_dig(j);
            int norm = decoder.get_channel_pwm(j);
            int fail = decoder.get_state();
        }
    }
    hal.console->printf("time AP_SBUS: %d, error frames: %d\n", AP_HAL::millis()-time, decoder.get_error_frames() );
    
    uint16_t values[18];
    uint16_t num_values[18];
    bool fail;
    bool drops;
    uint16_t max = 18;
    

    time = AP_HAL::millis();
    bool get;
    for(int i = 0; i < 10000000; i++) {
        get = sbus_decode(testarray2, values, num_values, &fail, &drops, max);
    }
    hal.console->printf("time AP: %d\n", AP_HAL::millis()-time);
}

AP_HAL_MAIN();
