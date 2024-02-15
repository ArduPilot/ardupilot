#pragma once

#include <time.h>

// replacement for mktime()
time_t ap_mktime(const struct tm *t);
