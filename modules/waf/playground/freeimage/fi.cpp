#include <stdio.h>
#include "FreeImage.h"

int main() {

#if defined(FREEIMAGE_LIB) || !defined(WIN32)
	FreeImage_Initialise(FALSE);
#endif

	FIBITMAP* dib = FreeImage_Load(FIF_PNG, "img.png", PNG_DEFAULT);
	printf("%dx%d", FreeImage_GetWidth(dib), FreeImage_GetHeight(dib));
	FreeImage_Unload(dib);

#if defined(FREEIMAGE_LIB) || !defined(WIN32)
	FreeImage_DeInitialise();
#endif

	return 0;
}
