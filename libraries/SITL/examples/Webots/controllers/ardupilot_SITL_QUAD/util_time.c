
#include <time.h>
#include <sys/time.h>
#include "util_time.h"

struct timespec getMilliseconds(void) {
    struct timespec ts;
    clock_gettime( CLOCK_REALTIME, &ts);
    return ts;
}
