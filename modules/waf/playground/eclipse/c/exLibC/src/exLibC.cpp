#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <pkg1/exLibC/exLibC.hpp>

int check_smaller(int value) {
	const char* foo = u8"bar";	// u8 is C++17 only
	std::cout << __cplusplus << std::endl; // Check version of C++ standard

	if (value < HELLO_LIMIT) {
		return 0;
	} else {
		return -1;
	}
}

