#include "time.h"

#include <stdint.h>

/*
  mktime replacement, originally from Samba
 */
time_t ap_mktime(const struct tm *t)
{
    // days elapsed from 1 January to the 1st of each month, common year
    static const uint16_t cumdays[12] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

    if (t->tm_year < 70) {
        return (time_t)-1;
    }

    /*
      every caller derives tm_mon from an external date field as
      "month - 1", so it can land outside 0-11 when the source reports
      a zero or out-of-range month.  Both cases are kept as the month
      loop this replaced behaved: it walked its own month and year
      counters forward, so months past December rolled into the
      following years, and a negative month added nothing because the
      loop simply never ran.
     */
    int mon = t->tm_mon;
    int year_off = t->tm_year;
    if (mon > 11) {
        year_off += mon / 12;
        mon %= 12;
    } else if (mon < 0) {
        mon = 0;
    }

    const uint32_t year = (uint32_t)year_off + 1900U;
    const uint32_t n = year - 1U;
    const bool leap = (year % 4) == 0 && ((year % 100) != 0 || (year % 400) == 0);

    // leap days since the epoch, as a closed form rather than a
    // per-year loop; 477 is the same count taken at 1969
    const uint32_t days = (uint32_t)(year_off - 70) * 365U
        + (n/4U - n/100U + n/400U) - 477U
        + cumdays[mon] + (leap && mon > 1)
        + (uint32_t)t->tm_mday - 1U;

    return (time_t)days * 86400 + t->tm_hour*3600 + t->tm_min*60 + t->tm_sec;
}
