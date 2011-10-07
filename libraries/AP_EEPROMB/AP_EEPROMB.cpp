/*
	RC_Channel.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com
	
	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include <avr/eeprom.h>
#include "AP_EEPROMB.h"

//#include "WProgram.h"

void
AP_EEPROMB::write_byte(int address, int8_t value)
{
	eeprom_write_byte((uint8_t *) address, value);
}

void
AP_EEPROMB::write_int(int address, int16_t value)
{
	eeprom_write_word((uint16_t *) address, value);
}

void
AP_EEPROMB::write_long(int address, int32_t value)
{
	eeprom_write_dword((uint32_t *) address, value);
}

void
AP_EEPROMB::write_float(int address, float value)
{
	_type_union.fvalue = value;
	write_long(address, _type_union.lvalue);
}

int
AP_EEPROMB::read_byte(int address)
{
	return eeprom_read_byte((const uint8_t *) address);
}

int
AP_EEPROMB::read_int(int address)
{
	return eeprom_read_word((const uint16_t *) address);
}

long
AP_EEPROMB::read_long(int address)
{
	return eeprom_read_dword((const uint32_t *) address);
}

float
AP_EEPROMB::read_float(int address)
{
	_type_union.lvalue = eeprom_read_dword((const uint32_t *) address);
	return _type_union.fvalue;
}


