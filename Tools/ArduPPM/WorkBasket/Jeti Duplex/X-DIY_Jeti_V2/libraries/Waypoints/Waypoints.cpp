/*
	AP_Radio.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com
	
	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include "Waypoints.h"

Waypoints::Waypoints()
{
}

void 
Waypoints::set_waypoint_with_index(Waypoints::WP wp, uint8_t i)
{
	i = constrain(i, 0, _total);
	uint32_t mem = _start_byte + (i * _wp_size);

	eeprom_busy_wait();
	eeprom_write_byte((uint8_t *) mem, wp.id);

	mem++;
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t *) mem, wp.p1);

	mem++;
	eeprom_busy_wait();
	eeprom_write_dword((uint32_t *) mem, wp.alt);

	mem += 4;
	eeprom_busy_wait();
	eeprom_write_dword((uint32_t *) mem, wp.lat);

	mem += 4;
	eeprom_busy_wait();
	eeprom_write_dword((uint32_t *) mem, wp.lng);
}

Waypoints::WP
Waypoints::get_waypoint_with_index(uint8_t i)
{
	Waypoints::WP wp;

	i = constrain(i, 0, _total);
	uint32_t mem = _start_byte + (i * _wp_size);
	
	eeprom_busy_wait();
	wp.id = eeprom_read_byte((uint8_t *)mem);

	mem++;
	eeprom_busy_wait();
	wp.p1 = eeprom_read_byte((uint8_t *)mem);

	mem++;
	eeprom_busy_wait();
	wp.alt = (long)eeprom_read_dword((uint32_t *)mem);

	mem += 4;
	eeprom_busy_wait();
	wp.lat = (long)eeprom_read_dword((uint32_t *)mem);

	mem += 4;
	eeprom_busy_wait();
	wp.lng = (long)eeprom_read_dword((uint32_t *)mem);
}


Waypoints::WP
Waypoints::get_current_waypoint(void)
{
	return get_waypoint_with_index(_index);
}

uint8_t
Waypoints::get_index(void)
{
	return _index;
}

void
Waypoints::next_index(void)
{
	_index++;
	if (_index >= _total)
		_index == 0;
}

void
Waypoints::set_index(uint8_t i)
{
	i = constrain(i, 0, _total);
}

uint8_t
Waypoints::get_total(void)
{
	return _total;
}
void
Waypoints::set_total(uint8_t total)
{
	_total = total;
}

void
Waypoints::set_start_byte(uint16_t start_byte)
{
	_start_byte = start_byte;
}

void
Waypoints::set_wp_size(uint8_t wp_size)
{
	_wp_size = wp_size;
}
