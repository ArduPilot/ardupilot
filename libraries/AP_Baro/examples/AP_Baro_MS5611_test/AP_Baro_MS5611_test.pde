
#include <stdint.h>
#include <FastSerial.h>
#include <SPI.h>
#include <AP_Baro.h> // ArduPilot Mega ADC Library

FastSerialPort0(Serial);

AP_Baro_MS5611 baro;

void setup()
{  
	Serial.begin(115200, 128, 128);
	Serial.println("ArduPilot Mega MeasSense Barometer library test");

	delay(1000);

  pinMode(63, OUTPUT);
  digitalWrite(63, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32); // 500khz for debugging, increase later

  baro.init();
}

void loop()
{
    int32_t pres;
    int32_t temp;

    Serial.println("Start Conversions");

    baro._start_conversion_D1();
    delay(10);
    bool res1 = baro._adc_read(&pres);
    baro._start_conversion_D2();
    delay(10);
    bool res2 = baro._adc_read(&temp);

    if (res1) {
        Serial.printf("Pressure raw value %ld\r\n",pres);
    } else {
        Serial.println("ADC conversion D1 unsuccessful");
    }

    if (res2) {
        Serial.printf("Temp raw value %ld\r\n",pres);
    } else {
        Serial.println("ADC conversion D2 unsuccessful");
    }
    
    Serial.println("---");
    delay(250);
}

void update_and_print()
{
    int32_t pres;
    float temp;

    baro.update();

    pres = baro.get_pressure();
    temp = baro.get_temp();
    Serial.printf("p: %ld t: %f \r\n", pres, temp);

    delay(100);
}
