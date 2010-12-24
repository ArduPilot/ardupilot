/*
 * Libraries
 */
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_EEProm.h>

AP_EEPromVar<int> var("TEST_VAR1");
AP_EEPromVar<float> var2("TEST_VAR2");
FastSerialPort0(Serial);

void setup()
{
	Serial.begin(115200);
	Serial.println("starting");
	displayMemory();
}

void loop()
{
	Serial.print("\n\nUn-synced variable demo (default)\n");

	var.setSync(false);
	var.set(123);
	Serial.printf_P(PSTR("\nvar.setSync(false): var.set(123): %d\n"), var.get());
	delay(2000);

	var.save();	
	var.set(456);
	Serial.printf_P(PSTR("\nvar.save(): var.set(456): %d\n"), var.get());
	delay(2000);

	var.load();
	Serial.printf_P(PSTR("\nval.load(): %d\n"), var.get());
	delay(2000);

	uint16_t id = var.getId();
	Serial.printf_P(PSTR("\nvar.getId(): %d\n"), id);
	delay(2000);

	Serial.printf_P(PSTR("\neepromRegistry(id)->getEntry(): %f\n"), eepromRegistry(id)->getEntry());
	delay(2000);

	eepromRegistry(id)->setEntry(456);
	Serial.printf_P(PSTR("\neepromRegistry(id)->setEntry(456): %d\n"), var.get());
	delay(2000);

	Serial.printf_P(PSTR("\nprint the parameters name by id: %s\n"), eepromRegistry(id)->getName());
	Serial.printf_P(PSTR("\nprint the parameters address by id: %d\n"), int(eepromRegistry(id)->getAddress()));


	Serial.print("\n\nSynced variable demo\n");

	var2.setSync(true);
	var2.set(1.23);
	Serial.printf_P(PSTR("\nvar2.setSync(false): var2.set(1.23): %f\n"), var2.get());
	delay(2000);

	var2.save();	
	var2.set(4.56);
	Serial.printf_P(PSTR("\nvar2.save(): var2.set(4.56): %f\n"), var2.get());
	delay(2000);

	var2.load();
	Serial.printf_P(PSTR("\nvar2.load(): %f\n"), var2.get());
	delay(2000);

	id = var2.getId();
	Serial.printf_P(PSTR("\nvar2.getId(): %d\n"), id);
	delay(2000);

	Serial.printf_P(PSTR("\neepromRegistry(id)->getEntry(): %f\n"), eepromRegistry(id)->getEntry());
	delay(2000);

	eepromRegistry(id)->setEntry(4.56);
	Serial.printf_P(PSTR("\neepromRegistry(id)->setEntry(4.56): %f\n"), var2.get());
	delay(2000);

	Serial.printf_P(PSTR("\nprint the parameters name by id: %s\n"), eepromRegistry(id)->getName());
	Serial.printf_P(PSTR("\nprint the parameters address by id: %d\n"), int(eepromRegistry(id)->getAddress()));

	delay(5000);
}
