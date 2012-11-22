/*
 *       Example of APM_RC library.
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 *
 *       Print Input values and send Output to the servos
 *       (Works with last PPM_encoder firmware)
 */

#include <FastSerial.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h> // ArduPilot Mega RC Library

FastSerialPort0(Serial);

Arduino_Mega_ISR_Registry isr_registry;
APM_RC_APM1 APM_RC;

void setup()
{
    isr_registry.init();
    APM_RC.Init(&isr_registry);          // APM Radio initialization
    APM_RC.enable_out(CH_1);
    APM_RC.enable_out(CH_2);
    APM_RC.enable_out(CH_3);
    APM_RC.enable_out(CH_4);
    APM_RC.enable_out(CH_5);
    APM_RC.enable_out(CH_6);
    APM_RC.enable_out(CH_7);
    APM_RC.enable_out(CH_8);

    Serial.begin(115200);
    Serial.println("ArduPilot Mega RC library test");
    delay(1000);
}

void loop()
{
    // New radio frame? (we could use also if((millis()- timer) > 20)
    if (APM_RC.GetState() == 1) {
        Serial.print("CH:");
        for(int i = 0; i < 8; i++) {
            Serial.print(APM_RC.InputCh(i));                    // Print channel values
            Serial.print(",");
            APM_RC.OutputCh(i, APM_RC.InputCh(i));             // Copy input to Servos
        }
        Serial.println();
    }
}
