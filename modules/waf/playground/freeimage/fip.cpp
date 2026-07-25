// run './waf --fip' to build this one

#include <iostream>
#include "FreeImagePlus.h"

int main() {

#if defined(FREEIMAGE_LIB) || !defined(WIN32)
	FreeImage_Initialise();
#endif

	fipImage img;
	img.load("img.png");
	std::cout << img.getWidth() << "x" << img.getHeight() << std::endl;

#if defined(FREEIMAGE_LIB) || !defined(WIN32)
	FreeImage_DeInitialise();
#endif

	return 0;
}
