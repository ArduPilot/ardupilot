#ifndef __AP_OSD_STREAM_H__
#define __AP_OSD_STREAM_H__

#include <AP_HAL.h>
#include "AP_HAL_Namespace.h"
#include "utility/BetterStream.h"

class AP_OSD_Stream : public AP_HAL::BetterStream
{
public:
	AP_OSD_Stream(){}

	/* Implementations of BetterStream virtual methods. These are
	* provided by AP_HAL to ensure consistency between ports to
	* different boards
	*/
	virtual void print_P(const prog_char_t *s);

	virtual void println_P(const prog_char_t *s);
	virtual void printf(const char *s, ...)
		__attribute__ ((format(__printf__, 2, 3)));
	virtual void _printf_P(const prog_char *s, ...)
		__attribute__ ((format(__printf__, 2, 3)));

	virtual void vprintf(const char *s, va_list ap);
	virtual void vprintf_P(const prog_char *s, va_list ap);

	virtual int16_t available();
	virtual int16_t txspace();
	virtual int16_t read();
	virtual size_t write(const uint8_t *buffer, size_t size);
	virtual size_t write(uint8_t c) = 0;
};

#endif //  __AP_OSD_STREAM_H__