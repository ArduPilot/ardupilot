/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * Storage.cpp --- AP_HAL_SMACCM storage driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 20 December 2012
 */

#include <AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM

#include <string.h>
#include <hwf4/eeprom.h>

#include "Storage.h"

using namespace SMACCM;

#define EEPROM_I2C_ADDR 0x50

// Note: These functions write multi-byte integers to the EEPROM in
// the native byte order, and so the format will depend on the
// endianness of the machine.

SMACCMStorage::SMACCMStorage()
{
}

void SMACCMStorage::init(void*)
{
  eeprom_init(i2c2, EEPROM_I2C_ADDR);
}

uint8_t SMACCMStorage::read_byte(uint16_t loc)
{
  uint8_t result = 0;
  eeprom_read_byte(loc, &result);
  return result;
}

uint16_t SMACCMStorage::read_word(uint16_t loc)
{
  uint16_t result = 0;
  eeprom_read(loc, (uint8_t*)&result, sizeof(result));
  return result;
}

uint32_t SMACCMStorage::read_dword(uint16_t loc)
{
  uint32_t result = 0;
  eeprom_read(loc, (uint8_t*)&result, sizeof(result));
  return result;
}

void SMACCMStorage::read_block(void* dst, uint16_t src, size_t n)
{
  eeprom_read(src, (uint8_t*)dst, n);
}

void SMACCMStorage::write_byte(uint16_t loc, uint8_t value)
{
  eeprom_write_byte(loc, value);
}

void SMACCMStorage::write_word(uint16_t loc, uint16_t value)
{
  eeprom_write(loc, (uint8_t*)&value, sizeof(value));
}

void SMACCMStorage::write_dword(uint16_t loc, uint32_t value)
{
  eeprom_write(loc, (uint8_t*)&value, sizeof(value));
}

void SMACCMStorage::write_block(uint16_t loc, void* src, size_t n)
{
  eeprom_write(loc, (const uint8_t *)src, n);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
