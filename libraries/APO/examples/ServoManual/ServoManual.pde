/*
 Example of RC_Channel library.
 Code by James Goppert/ Jason Short. 2010
 DIYDrones.com

 */
#include <FastSerial.h>
#include <AP_Common.h>
#include <APO.h>
#include <AP_RcChannel.h> 	// ArduPilot Mega RC Library
#include <APM_RC.h>
#include <AP_Vector.h>
FastSerialPort0(Serial)
; // make sure this proceeds variable declarations

// test settings

using namespace apo;

class RadioTest {
private:
	float testPosition;
	int8_t testSign;
	enum {
		version,
		rollKey,
		pitchKey,
		thrKey,
		yawKey,
		ch5Key,
		ch6Key,
		ch7Key,
		ch8Key
	};
	Vector<AP_RcChannel *> ch;
public:
	RadioTest() :
		testPosition(2), testSign(1) {
		ch.push_back(
				new AP_RcChannel(rollKey, PSTR("ROLL"), APM_RC, 0, 1100, 1500,
						1900));
		ch.push_back(
				new AP_RcChannel(pitchKey, PSTR("PITCH"), APM_RC, 1, 1100,
						1500, 1900));
		ch.push_back(
				new AP_RcChannel(thrKey, PSTR("THR"), APM_RC, 2, 1100, 1500,
						1900));
		ch.push_back(
				new AP_RcChannel(yawKey, PSTR("YAW"), APM_RC, 3, 1100, 1500,
						1900));
		ch.push_back(
				new AP_RcChannel(ch5Key, PSTR("CH5"), APM_RC, 4, 1100, 1500,
						1900));
		ch.push_back(
				new AP_RcChannel(ch6Key, PSTR("CH6"), APM_RC, 5, 1100, 1500,
						1900));
		ch.push_back(
				new AP_RcChannel(ch7Key, PSTR("CH7"), APM_RC, 6, 1100, 1500,
						1900));
		ch.push_back(
				new AP_RcChannel(ch8Key, PSTR("CH8"), APM_RC, 7, 1100, 1500,
						1900));

		Serial.begin(115200);
		delay(2000);
		Serial.println("ArduPilot RC Channel test");
		APM_RC.Init(); // APM Radio initialization
		delay(2000);
	}

	void update() {
		// read the radio 
		for (uint8_t i = 0; i < ch.getSize(); i++)
			ch[i]->setUsingRadio();

		// print channel names
		Serial.print("\t\t");
		static char name[7];
		for (uint8_t i = 0; i < ch.getSize(); i++) {
			ch[i]->copy_name(name, 7);
			Serial.printf("%7s\t", name);
		}
		Serial.println();

		// print pwm
		Serial.printf("pwm      :\t");
		for (uint8_t i = 0; i < ch.getSize(); i++)
			Serial.printf("%7d\t", ch[i]->getPwm());
		Serial.println();

		// print position
		Serial.printf("position :\t");
		for (uint8_t i = 0; i < ch.getSize(); i++)
			Serial.printf("%7.2f\t", ch[i]->getPosition());
		Serial.println();

		delay(500);
	}
};

RadioTest * test;

void setup() {
	test = new RadioTest;
}

void loop() {
	test->update();
}
