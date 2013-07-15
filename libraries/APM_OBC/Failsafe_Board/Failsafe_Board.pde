/*
  OBC failsafe board

  Jack Pittar and Andrew Tridgell

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public License
  as published by the Free Software Foundation; either version 2.1
  of the License, or (at your option) any later version.
*/

#define MODE_PWM_PIN 9 //Command pulse read on input 11
#define MUXA_PIN     3  // H H rc    , L H micro
#define MUXB_PIN     4  // H L backup, L L prime
#define LED_PIN      13
#define OBC_MODE_PIN 12 //high for OBC termination settings
#define AILERON_PIN  5
#define ELEVATOR_PIN 6
#define THROTTLE_PIN 7
#define RUDDER_PIN   8

#define HEARTBEAT_PRIMARY_PIN 11
#define TERMINATE_PRIMARY_PIN 10

#define HEARTBEAT_BACKUP_PIN A1
#define TERMINATE_BACKUP_PIN A0

// set to 1 if backup autopilot is installed
#define BACKUP_AUTOPILOT_INSTALLED 0

enum mux_mode {
	MUX_MODE_RC      = 1,
	MUX_MODE_MICRO   = 2,
	MUX_MODE_BACKUP  = 3,
	MUX_MODE_PRIMARY = 4
};

// heartbeat status
static bool heartbeat_primary_ok   = true;
static bool heartbeat_backup_ok    = true;



// check heartbeat status
static void update_heartbeat(void)
{
	static uint8_t last_hb_primary;
	static uint8_t last_hb_backup;
	static uint32_t last_primary_t;
	static uint32_t last_backup_t;
	uint8_t hb_primary   = digitalRead(HEARTBEAT_PRIMARY_PIN);
	uint8_t hb_backup    = digitalRead(HEARTBEAT_BACKUP_PIN);
	uint32_t tnow = millis();
//	Serial.println(hb_primary);
	// we consider the heartbeat to be OK if it changed in the
	// last 0.4 seconds
	const uint32_t hb_check_time = 400;
	if (hb_primary != last_hb_primary) {
		last_hb_primary = hb_primary;
		last_primary_t = tnow;
		//Serial.println("hb1 change");
	}
	if (hb_backup != last_hb_backup) {
		last_hb_backup = hb_backup;
		last_backup_t = tnow;
		//Serial.println("hb2 change");
	}
	if (tnow < hb_check_time) {
		// not enough time has passed for a measurement
		return;
	}
	heartbeat_primary_ok   = ((tnow - last_primary_t)   < hb_check_time);
	heartbeat_backup_ok = ((tnow - last_backup_t) < hb_check_time);
}


static void set_mux_mode(uint8_t mode)
{
	switch ((enum mux_mode)mode) {
	case MUX_MODE_RC:
		digitalWrite(MUXA_PIN, HIGH);
		digitalWrite(MUXB_PIN, HIGH);
		break;
	case MUX_MODE_MICRO:
		digitalWrite(MUXA_PIN, LOW);
		digitalWrite(MUXB_PIN, HIGH);
		break;
	case MUX_MODE_BACKUP:
		digitalWrite(MUXA_PIN, HIGH);
		digitalWrite(MUXB_PIN, LOW);
		break;
	case MUX_MODE_PRIMARY:
		digitalWrite(MUXA_PIN, LOW);
		digitalWrite(MUXB_PIN, LOW);
		break;		
	}
}

// send pulses to servos
static void set_servos(uint16_t aileron, uint16_t elevator,
		       uint16_t throttle, uint16_t rudder)
{
	digitalWrite(AILERON_PIN, HIGH);          
	delayMicroseconds(aileron);
	digitalWrite(AILERON_PIN, LOW);  

	digitalWrite(ELEVATOR_PIN, HIGH);
	delayMicroseconds(elevator);
	digitalWrite(ELEVATOR_PIN, LOW);

	digitalWrite(THROTTLE_PIN, HIGH);
	delayMicroseconds(throttle);
	digitalWrite(THROTTLE_PIN, LOW); 

	digitalWrite(RUDDER_PIN, HIGH);
	delayMicroseconds(rudder);
	digitalWrite(RUDDER_PIN, LOW);
}

static void set_servos_terminate(uint8_t obc_mode)
{
	set_mux_mode(MUX_MODE_MICRO);
	if (obc_mode) {
		set_servos(1000, 2000, 1000, 1000);
	} else {
		set_servos(1500, 1500, 1200, 1500);
	}
}


void setup() 
{
	// input pins
	pinMode(HEARTBEAT_PRIMARY_PIN, INPUT);
	pinMode(TERMINATE_PRIMARY_PIN, INPUT);
	pinMode(HEARTBEAT_BACKUP_PIN, INPUT);
	pinMode(TERMINATE_BACKUP_PIN, INPUT);
	pinMode(MODE_PWM_PIN, INPUT);
	pinMode(OBC_MODE_PIN, INPUT);

	// output pins
	pinMode(LED_PIN, OUTPUT);
	pinMode(MUXA_PIN, OUTPUT);
	pinMode(MUXB_PIN, OUTPUT);
	pinMode(AILERON_PIN, OUTPUT); 
	pinMode(ELEVATOR_PIN, OUTPUT);
	pinMode(THROTTLE_PIN, OUTPUT);
	pinMode(RUDDER_PIN, OUTPUT);   

	digitalWrite(LED_PIN, LOW);
	set_mux_mode(MUX_MODE_MICRO);

	Serial.begin(115200);
	Serial.println("OBC Failsafe Starting\n");
}   

void loop() 
{
	static uint32_t last_status_t;
	uint32_t tnow = millis();
	static uint8_t led_state;
	static bool has_terminated = false;
	static uint8_t termination_counter;
	static uint16_t loop_counter;

	loop_counter++;

	// check for heartbeat
	update_heartbeat();

	// see if we are in manual mode. Note that on the Uno,
	// pulseIn is not very accurate. The +90 brings us much closer
	// to the real pulse width
	uint16_t mode_pwm = pulseIn(MODE_PWM_PIN, HIGH, 25000) + 90;
	uint16_t manual_mode = (mode_pwm > 1750 && mode_pwm < 2100);

	// check the state of the terminate pins
	uint8_t terminate_primary = digitalRead(TERMINATE_PRIMARY_PIN);
	uint8_t terminate_backup  = digitalRead(TERMINATE_BACKUP_PIN);

	// see if the OBC mode jumper is set
	uint8_t obc_mode = digitalRead(OBC_MODE_PIN);

	if (tnow - last_status_t > 1000) {
		last_status_t = tnow;
		// show status once a second
		Serial.print("OBC:"); Serial.print(obc_mode);
		Serial.print(" Mode:"); Serial.print(mode_pwm);
		Serial.print(" HB1:"); Serial.print(heartbeat_primary_ok);
		Serial.print(" HB2:"); Serial.print(heartbeat_backup_ok);
		Serial.print(" TERM1:"); Serial.print(terminate_primary);
		Serial.print(" TERM2:"); Serial.print(terminate_backup);
		Serial.print(" TERMINATED:"); Serial.print(has_terminated);
		Serial.print(" LOOP:"); Serial.print(loop_counter);
		Serial.println();
		delayMicroseconds(5000);
		loop_counter = 0;
		// flash LED once a second so we know failsafe board
		// is working
		led_state = !led_state;
		digitalWrite(LED_PIN, led_state);
	}

	// allow reset via serial connection
	if (Serial.available() > 0) {
		char c = (char)Serial.read();
		if (c == 'r') {
			Serial.println("RESET BOARD");
			has_terminated = false;
			termination_counter = 0;
		}
	}

	// if we are not in OBC mode, and the mode control 
	// channel is high, then give RC control
	if (!obc_mode && manual_mode) {
		// give manual control via RC
		set_mux_mode(MUX_MODE_RC);		
		return;
	}

#if BACKUP_AUTOPILOT_INSTALLED == 0
	terminate_backup = false;
#endif

	// see if termination is set by a functioning autopilot
	if (heartbeat_primary_ok && terminate_primary) {
		termination_counter++;
	} else if (heartbeat_backup_ok && terminate_backup) {
		termination_counter++;
	} else if (obc_mode && !heartbeat_primary_ok && !heartbeat_backup_ok) {
		// if in OBC mode and neither autopilot is OK, then
		// terminate
		termination_counter++;
	} else {
		termination_counter = 0;
	}

	// use the termination counter to debounce the termination
	// pins
	if (termination_counter > 10) {
		termination_counter = 0;
		has_terminated = true;
	}

	// if we have terminated then setup the servos
	if (has_terminated) {
		set_servos_terminate(obc_mode);
		return;		
	}

	bool heartbeat_ok = heartbeat_primary_ok;
#if BACKUP_AUTOPILOT_INSTALLED
	if (heartbeat_backup_ok) {
		heartbeat_ok = true;
	}
#endif

	if (heartbeat_ok) {
		// at least one autopilot is healthy
		if (manual_mode) {
			// we want manual/RC control
			set_mux_mode(MUX_MODE_RC);
		} else if (heartbeat_primary_ok) {
			set_mux_mode(MUX_MODE_PRIMARY);
		} else {
			set_mux_mode(MUX_MODE_BACKUP);
		}
	} else {
		// neither autopilot is OK
		if (obc_mode) {
			set_servos_terminate(obc_mode);
		} else {
			// give RC control 
			set_mux_mode(MUX_MODE_RC);
		}
	}
}

