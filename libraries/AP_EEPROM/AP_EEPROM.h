// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_EEPROM.h
/// @brief	AP_EEPROM manager

#ifndef AP_EEPROM_h
#define AP_EEPROM_h

//#include <stdint.h>
#include "WProgram.h"

/// @class	AP_EEPROM
/// @brief	Object for reading and writing to the EEPROM
class AP_EEPROM{
  public:	
	/// Constructor
	AP_EEPROM(){}

	int		read_byte(int address);
	int		read_int(int address);
	long	read_long(int address);
	float	read_float(int address);
	
	void	write_byte(int address, int8_t value);
	void	write_int(int address, int16_t value);
	void	write_long(int address, int32_t value);
	void	write_float(int address, float value);
	
  private:
  
	union type_union {
		int8_t	bytes[4];
		long 	lvalue;
		int 	ivalue;
		float 	fvalue;
	} _type_union;
  
};

#endif	

