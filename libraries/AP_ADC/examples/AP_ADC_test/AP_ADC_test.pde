
/*
 *  Example of APM_ADC library.
 *  Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 */

#include <FastSerial.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_ADC.h> // ArduPilot Mega ADC Library

FastSerialPort0(Serial);        // FTDI/console

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess adc_scheduler;


unsigned long timer;
AP_ADC_ADS7844 adc;

void setup()
{
    Serial.begin(115200, 128, 128);
    Serial.println("ArduPilot Mega ADC library test");
    delay(1000);

    isr_registry.init();
    adc_scheduler.init(&isr_registry);

    adc.Init(&adc_scheduler);       // APM ADC initialization
    delay(1000);
    timer = millis();
}

static const uint8_t channel_map[6] = { 1, 2, 0, 4, 5, 6};
float v;
uint32_t last_usec = 0;

static void show_timing()
{
    uint32_t mint = (uint32_t)-1, maxt = 0, totalt=0;
    uint32_t start_time = millis();
    uint16_t result[6];
    uint32_t count = 0;

    Serial.println("Starting timing test");

    adc.Ch6(channel_map, result);

    do {
        uint32_t deltat = adc.Ch6(channel_map, result);
        if (deltat > maxt) maxt = deltat;
        if (deltat < mint) mint = deltat;
        totalt += deltat;
        count++;
    } while ((millis() - start_time) < 5000);

    Serial.printf("timing: mint=%lu maxt=%lu avg=%lu\n", mint, maxt, totalt/count);
}

static void show_data()
{
    uint16_t result[6];
    uint32_t deltat = 0;
    uint16_t ch3;
    uint16_t min[6], max[6];
    uint8_t i;

    for (i=0; i<6; i++) {
        min[i] = 0xFFFF;
        max[i] = 0;
    }


    do {
        ch3 = adc.Ch(3);
        deltat += adc.Ch6(channel_map, result);
        for (i=0; i<6; i++) {
            if (result[i] < min[i]) min[i] = result[i];
            if (result[i] > max[i]) max[i] = result[i];
            if (result[i] & 0x8000) {
                Serial.printf("result[%u]=0x%04x\n", (unsigned)i, result[i]);
            }
        }
    } while ((millis() - timer) < 200);

    timer = millis();
    Serial.printf("g=(%u,%u,%u) a=(%u,%u,%u) +/-(%u,%u,%u,%u,%u,%u) gt=%u dt=%u\n",
                  result[0], result[1], result[2],
                  result[3], result[4], result[5],
                  (max[0]-min[0]), (max[1]-min[1]), (max[2]-min[2]),
                  (max[3]-min[3]), (max[4]-min[4]), (max[5]-min[5]),
                  ch3, (unsigned)(deltat/1000));
}


void loop()
{
    if (millis() < 5000) {
        show_timing();
    } else {
        show_data();
    }
}
