/*
    forced to include to ALL files

*/

using namespace std;

#ifndef __EXPORT
#define __EXPORT __attribute__ ((visibility ("default")))
#endif

#ifndef __PRIVATE
#define __PRIVATE __attribute__ ((visibility ("hidden")))
#endif

#ifndef packed_struct
#define packed_struct __attribute__ ((packed))
#endif

#define _WANT_REGISTER_FINI 1
#define HAVE_INITFINI_ARRAY 1
