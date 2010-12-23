/*
 * Libraries
 */
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_EEProm.h>

AP_EEPromVar<int> var;
FastSerialPort0(Serial);

void setup()
{
	Serial.begin(115200);
	Serial.println("starting");
	displayMemory();
}

void loop()
{
	var.set(123);
	Serial.printf_P(PSTR("initially set 123 and save: %d\n"), var.get());
	var.save();	
	var.set(456);
	Serial.printf_P(PSTR("next set to 456: %d\n"), var.get());
	var.load();
	Serial.printf_P(PSTR("now reload initial value: %d\n"), var.get());
	uint16_t id = var.getId();
	Serial.printf_P(PSTR("now find id for variable: %d\n"), id);
	Serial.printf_P(PSTR("now find variable value by id: %f\n"), eepromRegistry(id)->getEntry());
	eepromRegistry(id)->setEntry(456);
	Serial.printf_P(PSTR("now set variable value by id to 456: %d\n"), var.get());
	delay(5000);
}
