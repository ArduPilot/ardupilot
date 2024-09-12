#include "time.h"

/*
  mktime replacement from Samba
 */
time_t ap_mktime(const struct tm *t)
{
    time_t epoch = 0;
    int n;
    int mon [] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }, y, m, i;
    const unsigned MINUTE = 60;
    const unsigned HOUR = 60*MINUTE;
    const unsigned DAY = 24*HOUR;
    const unsigned YEAR = 365*DAY;

    if (t->tm_year < 70) {
        return (time_t)-1;
    }

    n = t->tm_year + 1900 - 1;
    epoch = (t->tm_year - 70) * YEAR +
            ((n / 4 - n / 100 + n / 400) - (1969 / 4 - 1969 / 100 + 1969 / 400)) * DAY;

    y = t->tm_year + 1900;
    m = 0;

    for (i = 0; i < t->tm_mon; i++) {
        epoch += mon [m] * DAY;
        if (m == 1 && y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) {
            epoch += DAY;
        }

        if (++m > 11) {
            m = 0;
            y++;
        }
    }

    epoch += (t->tm_mday - 1) * DAY;
    epoch += t->tm_hour * HOUR + t->tm_min * MINUTE + t->tm_sec;

    return epoch;
}

