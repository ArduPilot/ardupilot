/*
	AP_Radio.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com
	
	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include "Waypoints.h"

Waypoints::Waypoints(uint16_t start_byte, uint8_t wp_size, uint8_t total)
{
	_start_byte	= start_byte;
	_wp_size	= wp_size;
	_total		= total;
}

void 
Waypoints::set_waypoint_with_index(Waypoints::WP wp, uint16_t i)
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
Waypoints::get_waypoint_with_index(uint16_t i)
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
Waypoints::get_next_waypoint(void)
{
	_index++;
	if (_index >= _total)
		_index == 0;
}

uint8_t
Waypoints::get_index(void)
{
	return _index;
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
