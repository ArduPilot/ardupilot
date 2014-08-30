/*
 * util_stub.h
 *
 *  Created on: 28 mai 2014
 *      Author: valentin
 */

#ifndef UTIL_STUB_H_
#define UTIL_STUB_H_

#include "util.h"

class Util {
public:
	int vsnprintf_P(char* str, size_t size, const prog_char_t *format, va_list ap) {return 1;}
};

#endif /* UTIL_STUB_H_ */
