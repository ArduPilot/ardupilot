/*
 * Libraries
 */
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_EEProm.h>

AP_EEPromVar<int> var(1,"TEST_VAR1");
AP_EEPromVar<float> var2(2.0,"TEST_VAR2");
AP_EEPromVar<int16_t> var3(-700,"TEST_VAR3");
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

	uint16_t id = var.getEntryId();
	Serial.printf_P(PSTR("\nvar.getEntryId(): %d\n"), id);
	delay(2000);

	Serial.printf_P(PSTR("\neepromRegistry(id)->getEntry(): %f\n"), eepromRegistry[id]->getEntry());
	delay(2000);

	eepromRegistry[id]->setEntry(456);
	Serial.printf_P(PSTR("\neepromRegistry(id)->setEntry(456): %d\n"), var.get());
	delay(2000);

	Serial.printf_P(PSTR("\nprint the parameters name by id: %s\n"), eepromRegistry[id]->getEntryName());
	delay(2000);

	Serial.printf_P(PSTR("\nprint the parameters address by id: %d\n"), int(eepromRegistry[id]->getEntryAddress()));
	delay(2000);


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

	id = var2.getEntryId();
	Serial.printf_P(PSTR("\nvar2.getEntryId(): %d\n"), id);
	delay(2000);

	Serial.printf_P(PSTR("\neepromRegistry(id)->getEntry(): %f\n"), eepromRegistry[id]->getEntry());
	delay(2000);

	eepromRegistry[id]->setEntry(4.56);
	Serial.printf_P(PSTR("\neepromRegistry(id)->setEntry(4.56): %f\n"), var2.get());
	delay(2000);

	Serial.printf_P(PSTR("\nprint the parameters name by id: %s\n"), eepromRegistry[id]->getEntryName());
	delay(2000);

	Serial.printf_P(PSTR("\nprint the parameters address by id: %d\n"), int(eepromRegistry[id]->getEntryAddress()));

	delay(5000);
}
