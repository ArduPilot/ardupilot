/*
 * AP_EEProm.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_EEProm.h>

void AP_EEPromRegistry::add(AP_EEPromEntry * entry, uint16_t & id, uint16_t & address, size_t size)
{
	if (_newAddress + size > _maxSize) return;
	address = _newAddress;
	_newAddress += size;
	id = _newId++;
	push_back(entry);
}

/**
 * The global declaration for the eepromRegistry
 */
extern AP_EEPromRegistry eepromRegistry(1024);
