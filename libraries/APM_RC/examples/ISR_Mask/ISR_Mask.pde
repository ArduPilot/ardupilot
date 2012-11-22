/*
  test interrupt masking methods on APM1
 */

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h>

FastSerialPort0(Serial);

Arduino_Mega_ISR_Registry isr_registry;
APM_RC_APM1 APM_RC;

void setup()
{
    Serial.begin(115200);
    isr_registry.init();
    APM_RC.Init(&isr_registry);
    Serial.println("Interrupt masking test");
    delay(10);
}

static uint16_t last_values[8];

#define USE_CLI  0
#define USE_MASK 1

void loop()
{
	if (APM_RC.GetState() == 1) {
		bool changed = false;
		for (uint8_t i = 0; i < 8; i++) {
			uint16_t v = APM_RC.InputCh(i);
			if (abs(v - last_values[i]) > 10) {
				changed = true;
			}
			last_values[i] = v;
		}
		if (changed) {
			for (uint8_t i = 0; i < 8; i++) {
				Serial.printf("%u:%4u ", (unsigned)i, (unsigned)last_values[i]);
			}
			Serial.println();
			delayMicroseconds(500);
		}
	}

#if USE_CLI
	uint8_t oldSREG = SREG;
	cli();
	delayMicroseconds(100);
	SREG = oldSREG;
#endif

#if USE_MASK
	uint8_t _timsk4 = TIMSK4;
	TIMSK4 &= ~(1<<ICIE4);
	delayMicroseconds(100);
	TIMSK4 = _timsk4;
#endif
}
