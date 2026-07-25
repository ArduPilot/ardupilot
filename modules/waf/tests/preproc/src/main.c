
#if A
const char* c = "foo /*";
  #if B
const char* d = " bar */";
// /*
    #if C
// */
      #include "a.h"
    #else
      #include "b.h"
    #endif
  #else
    #if C
      #include "c.h"
    #else
      #include "d.h"
    #endif
  #endif
#elif B
  #if C
	# if 1 - 1
		#include "a.h"
	# endif
	# if 0
	#include "a.h"
	# endif
     #include <e.h>
  #else
     #include "f.h"
  #endif
#elif C
  #include "g.h"
#else
  #include "h.h"
#endif


/*
#include "a.h"
*/
//#include "a.h"

int main() {
	return 0;
}
