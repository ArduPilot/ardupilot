/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
*/

// Libraries
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h>         // ArduPilot Mega RC Library

////////////////////////////////////////////////////////////////////////////////
// Serial ports
FastSerialPort0(Serial);        // FTDI/console

APM_RC_APM2 APM_RC;
Arduino_Mega_ISR_Registry isr_registry;

#define HEARTBEAT_PIN A0
#define FAILSAFE_PIN  A1

#define COMPETITION_MODE 0

void setup() {
	Serial.begin(115200, 256, 256);
	isr_registry.init();
	APM_RC.Init(&isr_registry);		// APM Radio initialization
	pinMode(HEARTBEAT_PIN, INPUT);
	pinMode(FAILSAFE_PIN, INPUT);

	// enable all output channels
	for (uint8_t ch=0; ch<8; ch++) {
		APM_RC.enable_out(ch);
	}
	Serial.printf("Failsafe board started\n");
}    


static uint8_t last_heartbeat;
static uint32_t last_heartbeat_change_t;
static bool heartbeat_failure;
static uint32_t last_print;

void loop()
{
	uint16_t pwm[8];
	bool failsafe_enabled;
	uint32_t tnow = millis();
	uint8_t heartbeat_pin = digitalRead(HEARTBEAT_PIN);

    // see if we have a new radio frame
	if (APM_RC.GetState() != 1) {
        return;
    }

	if (heartbeat_pin != last_heartbeat) {
		last_heartbeat = heartbeat_pin;
		last_heartbeat_change_t = tnow;
	}
	bool hb_failed = (tnow - last_heartbeat_change_t > 500);
	if (hb_failed != heartbeat_failure) {
		heartbeat_failure = hb_failed;
		Serial.printf("HBFAIL=%u\n", (unsigned)heartbeat_failure);
	}

	// ch0 to ch3 come from the autopilot
	// ch4 to ch7 come from the receiver

	for (uint8_t ch=0; ch<8; ch++) {
		pwm[ch] = APM_RC.InputCh(ch);
	}
	failsafe_enabled = digitalRead(FAILSAFE_PIN);
	if (tnow - last_print > 200) {
		Serial.printf("1:%4u 2:%4u 3:%4u 4:%4u 5:%4u 6:%4u 7:%4u 8:%4u HB=%u FS=%u HBFAIL=%u\n",
			      pwm[0], pwm[1], pwm[2], pwm[3], 
			      pwm[4], pwm[5], pwm[6], pwm[7],
			      (unsigned)digitalRead(HEARTBEAT_PIN),
			      (unsigned)failsafe_enabled,
			      (unsigned)heartbeat_failure);
		last_print = tnow;
	}
	if (failsafe_enabled || heartbeat_failure) {
#if COMPETITION_MODE
		// if we are in failover, send receiver input straight
		// to the servos
		for (uint8_t ch=0; ch<4; ch++) {
			APM_RC.OutputCh(ch, 1000);
		}
#else
		// if we are in failover, send receiver input straight
		// to the servos
		for (uint8_t ch=0; ch<4; ch++) {
			APM_RC.OutputCh(ch, pwm[ch+4]);
		}
#endif
	} else {
		// otherwise send autopilot output to the servos
		for (uint8_t ch=0; ch<4; ch++) {
			APM_RC.OutputCh(ch, pwm[ch]);
		}
	}
	// always pass the receiver input to the autopilot
	for (uint8_t ch=4; ch<8; ch++) {
		APM_RC.OutputCh(ch, pwm[ch]);
	}
}
