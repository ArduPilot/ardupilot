#include "mathutilities.h"

//! Number of days between Jan 6 1980 and Jan 1 2012
#define JAN12012 11683

//! Day number at start of each month for a common year, day and month are zero based
static const uint16_t month_day_norm[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

//! Day number at start of each month for a leap year, day and month are zero based
static const uint16_t month_day_leap[12] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};

//! Determine if a year is a leap year in the Gregorian calendar.
int isLeapYear(uint16_t year);


/*!
 * Add two angles together accounting for circular wrap
 * \param first is the first angle in radians in the range -PI to PI
 * \param second is the second angle in radians in the range -PI to PI
 * \return the correctly wrapped sum of first and second, in the range -PI to PI
 */
double addAngles(double first, double second)
{
    return wrapAngle(first + second);
}


/*!
 * Subtract one angle from another accounting for circular wrap
 * \param left is the first angle in radians in the range -PI to PI
 * \param serightcond is the second angle in radians in the range -PI to PI
 * \return the correctly wrapped difference of left minus right, in the range -PI to PI
 */
double subtractAngles(double left, double right)
{
    return wrapAngle(left - right);
}


/*!
 * Adjust an angle for circular wrap. The input angle will only be adjusted by
 * one circle (2PI). Arbitrary angles should be adjusted using fmod(angle, 2PI)
 * \param angle is the angle to adjust in radians, in the range of -3PI to 3PI.
 * \return an equivalent angle in the range of -PI to PI.
 */
double wrapAngle(double angle)
{
    if(angle > PId)
        angle -= 2*PId;
    else if(angle <= -PId)
        angle += 2*PId;

    return angle;
}


/*!
 * Adjust an angle for circular wrap with the wrap point at -270/+90. The input
 * angle will only be adjusted by one circle (2PI).
 * \param angle is the angle to adjust in radians, in the range of -3.5PI to 2.5PI.
 * \return an equivalent angle in the range of -1.5PI to 0.5PI.
 */
double wrapAngle90(double angle)
{
    if(angle > PId/2)
        angle -= 2*PId;
    else if(angle <= -3*PId/2)
        angle += 2*PId;

    return angle;
}


/*!
 * A simple first order low pass filter where state is stored by the caller.
 * \param prev is the previous output of the filter
 * \param sig is the new signal
 * \param tau is the filter time constant
 * \param sampleTime is the iteration period of the filter, in the same units as tau
 * \return the new filtered value, which the caller must store
 */
double firstOrderFilter(double prev, double sig, double tau, double sampleTime)
{
    return prev + sampleTime * (sig - prev) / (tau + sampleTime);

}// firstOrderFilter


/*!
 * Add two angles together accounting for circular wrap
 * \param first is the first angle in radians in the range -PI to PI
 * \param second is the second angle in radians in the range -PI to PI
 * \return the correctly wrapped sum of first and second, in the range -PI to PI
 */
float addAnglesf(float first, float second)
{
    return wrapAnglef(first + second);
}


/*!
 * Subtract one angle from another accounting for circular wrap
 * \param left is the first angle in radians in the range -PI to PI
 * \param serightcond is the second angle in radians in the range -PI to PI
 * \return the correctly wrapped difference of left minus right, in the range -PI to PI
 */
float subtractAnglesf(float left, float right)
{
    return wrapAnglef(left - right);
}


/*!
 * Adjust an angle for circular wrap. The input angle will only be adjusted by
 * one circle (2PI). Arbitrary angles should be adjusted using fmod(angle, 2PI)
 * \param angle is the angle to adjust in radians, in the range of -3PI to 3PI.
 * \return an equivalent angle in the range of -PI to PI.
 */
float wrapAnglef(float angle)
{
    if(angle > PIf)
        angle -= 2*PIf;
    else if(angle <= -PIf)
        angle += 2*PIf;

    return angle;
}


/*!
 * Adjust an angle for circular wrap with the wrap point at -270/+90. The input
 * angle will only be adjusted by one circle (2PI).
 * \param angle is the angle to adjust in radians, in the range of -3.5PI to 2.5PI.
 * \return an equivalent angle in the range of -1.5PI to 0.5PI.
 */
float wrapAngle90f(float angle)
{
    if(angle > PIf/2)
        angle -= 2*PIf;
    else if(angle <= -3*PIf/2)
        angle += 2*PIf;

    return angle;
}


/*!
 * Adjust an angle for circular wrap with the wrap point at 0/360. The input
 * angle will only be adjusted by one circle (2PI).
 * \param angle is the angle to adjust in radians, in the range of -2PI to 2PI.
 * \return an equivalent angle in the range of 0 to 2PI.
 */
float wrapAngle360f(float angle)
{
    if(angle < 0)
        angle += 2*PIf;

    return angle;
}


/*!
 * Compute a fast approximation to the sine of an angle
 * \param angle is the angle to compute the sine of, in radians
 * \return the approximate sine of angle
 */
float fastSin(float angle)
{
    // Quasi-Taylor expansion coefficients
    static const float A[] = { 1 / 6.0f, 1 / 20.0f, 1 / 42.0f, 1 / 72.0f };
    float X;

    // Wrap to +/-pi
    angle = wrapAnglef(angle);

    // Pre-compute angle^2
    X = angle * angle;

    // Return a simplified version of the Taylor expansion of sin(x)
    return angle * (1 - A[0]*X * (1 - A[1]*X * (1 - A[2]*X * (1 - A[3]*X))));

}// fastSin


/*!
 * Compute a fast approximation to the cosine of an angle
 * \param angle is the angle to compute the cosine of, in radians
 * \return the approximate cosine of angle
 */
float fastCos(float angle)
{
    // Quasi-Taylor expansion coefficients
    static const float A[] = { 1 / 2.0f, 1 / 12.0f, 1 / 30.0f, 1 / 56.0f, 1 / 90.0f };
    float X;

    // Wrap to +/-pi
    angle = wrapAnglef(angle);

    // Pre-compute angle^2
    X = angle * angle;

    // Return a simplified version of the Taylor expansion of cos(x)
    return 1 - A[0]*X * (1 - A[1]*X * (1 - A[2]*X * (1 - A[3]*X * (1 - A[4]*X))));

}// fastCos


/*! 
 * Fast inverse square root approximation
 * \param x is the number to take the inverse square root of
 * \return x ^ -0.5
 */
float fastISqrt(float x)
{
    // Union for reinterpreting x as an int32_t
    union { float f; int32_t i; } u;

    // "What the fuck?" --John Carmack
    u.f = x;
    u.i = 0x5f3759dfL - (u.i >> 1);

    // Single Newton iteration
    return u.f * (1.5f - (0.5f * x * u.f * u.f));

}// fastISqrt


/*! 
 * Fast square root approximation
 * \param x is the number to take the square root of
 * \return x ^ 0.5
 */
float fastSqrt(float x)
{
    // Multiply inverse square root by X to get sqrt(x)
    return x * fastISqrt(x);

}// fastSqrt


/*!
 * A simple first order low pass filter where state is stored by the caller.
 * \param prev is the previous output of the filter
 * \param sig is the new signal
 * \param tau is the filter time constant
 * \param sampleTime is the iteration period of the filter, in the same units as tau
 * \return the new filtered value, which the caller must store
 */
float firstOrderFilterf(float prev, float sig, float tau, float sampleTime)
{
    return prev + sampleTime * (sig - prev) / (tau + sampleTime);

}// firstOrderFilterf


/*!
 * Apply a rate of change limit
 * \param prev is the previous output of the limiter
 * \param value is the new proposed value whose derivative should be limited
 * \param limit is the time-rate-of-change limit for value, i.e. the max time derivative of value.
 * \param sampleTime is the elapsed time since prev was computed.
 * \return The rate-of-change limited value.
 */
float rateOfChangeLimitf(float prev, float value, float limit, float sampleTime)
{
    float delta;

    // the maximum amount of change allowed
    float maxDelta = sampleTime*limit;

    // Which must be positive non-zero to make sense
    if(maxDelta <= 0)
        return value;

    // The actual proposed changed
    delta = value-prev;

    // Return the limited value if needed
    if(delta > maxDelta)
        return prev + maxDelta;
    else if(delta < -maxDelta)
        return prev - maxDelta;
    else
        return value;

}


/*!
 * Use GPS time information to compute the Gregorian calendar date and time with respect to UTC.
 * This only works for dates after Jan 1 2012.
 * \param week is the GPS week number
 * \param itow is the GPS time of week in milliseconds
 * \param leapsecs is the number of leapseconds to subtract from GPS time to get UTC time
 * \param pyear receives the Gregorian year
 * \param pmonth receives the month of the year, from 1 (January) to 12 (December)
 * \param pday receives the day of the month, from 1 to 31
 * \param phour receives the hour of the day from 0 to 23
 * \param pmin receives the minute of the hour from 0 to 59
 * \param psec receives the seconds of the minute from 0 to 59
 */
void computeDateAndTimeFromWeekAndItow(uint16_t week, uint32_t itow, uint8_t leapsecs, uint16_t* pyear, uint8_t* pmonth, uint8_t* pday, uint8_t* phour, uint8_t* pmin, uint8_t* psec)
{
    // Watch for wrap when we subtract the leap seconds
    if(itow < leapsecs*1000)
    {
        week--;
        itow += 86400000*7;
    }

    // Account for leap seconds, this converts the GPS time to UTC time
    itow -= leapsecs*1000;

    // compute the date
    computeDateFromWeekAndItow(week, itow, pyear, pmonth, pday);

    // And the time
    computeTimeFromItow(itow, phour, pmin, psec);

}// computeDateAndtimeFromWeekAndItow


/*!
 * Use GPS time information to compute the Gregorian calendar date.
 * This only works for dates after Jan 1 2012.
 * \param week is the GPS week number
 * \param itow is the GPS time of week in milliseconds
 * \param pyear receives the Gregorian year
 * \param pmonth receives the month of the year, from 1 (January) to 12 (December)
 * \param pday receives the day of the month, from 1 to 31
 */
void computeDateFromWeekAndItow(uint16_t week, uint32_t itow, uint16_t* pyear, uint8_t* pmonth, uint8_t* pday)
{
    // If the date is after 1/1/2012
    if (week >= JAN12012 / 7)
    {
        int isleap;
        uint8_t   month;
        const uint16_t* month_day;
        uint16_t dayinyear;

        // We don't support dates before this year
        uint16_t year = 2012;

        // Number of days since Jan 6 1980.
        uint32_t days = week*7 + itow/86400000;

        // Convert to number of days since jan 1 2012
        days -= JAN12012;

        // The number of days in a year depends on if it is a leap year
        isleap = isLeapYear(year);
        if(isleap)
            dayinyear = 366;
        else
            dayinyear = 365;

        while(days >= dayinyear)
        {
            // subtract off the days
            days -= dayinyear;

            // Go to the next year
            year++;

            isleap = isLeapYear(year);
            if(isleap)
                dayinyear = 366;
            else
                dayinyear = 365;

        }// while finding the year

        // now we know the year
        *pyear = year;

        // The month_day list is leap year dependent
        if(isleap)
            month_day = month_day_leap;
        else
            month_day = month_day_norm;

        // Now we have the year, we need to get the month.
        for(month = 1; month < 12; month++)
        {
            if(days < month_day[month])
                break;
        }

        // i is the zero-based month from 1(feb) to 12(dec+1), whose starting day is after
        // the days, we want the zero based month from 0 to 11 whose starting
        // day is just before or equal to days
        month--;

        // Subtract off the number of days for the month
        days -= month_day[month];

        // Record days and months, notice they are 1-based
        *pday = days+1;
        *pmonth = month+1;
    }
    else
    {
        // Otherwise, party like it's 1980
        *pyear = 1980;
        *pmonth = 1;
        *pday = 6;
    }

}// computeDateFromWeekAndITOW


/*!
 * Compute hours, minutes, and seconds from time of week.
 * \param itow is the time of week in milliseconds
 * \param hour receives the hour of the day from 0 to 23
 * \param min receives the minute of the hour from 0 to 59
 * \param second receives the seconds of the minute from 0 to 59
 */
void computeTimeFromItow(uint32_t itow, uint8_t* hour, uint8_t* min, uint8_t* second)
{
    // Milliseconds of the day
    itow = itow % 86400000;

    // Compute hours of the day
    (*hour) = (uint8_t)(itow / (60UL * 60UL * 1000UL));

    // Subtract off hours
    itow -= (*hour)*60*60*1000;

    // Compute minutes of the hour
    (*min)  = (uint8_t)(itow / (60UL * 1000UL));

    // Subtract off minutes
    itow -= (*min)*60*1000;

    // Compute seconds of the minute
    (*second)  = (uint8_t)(itow / (1000UL));

}// computeTimeFromItow


/*!
 * Determine if a year is a leap year in the Gregorian calendar.
 * \param year is the year number.
 * \return 0 for a common year (365 days), 1 for a leap year (366 days)
 */
int isLeapYear(uint16_t year)
{
    // Leap year rules:
    // 1) every 4th year is a leap year unless:
    // 2) the year is modulo 100 its not a leap year, unless:
    // 3) the year is module 400 it is a leap year. So:
    // 1896 is leap year,
    // 1900 is not (modulo 100),
    // 1904 is
    // 1996 is
    // 2000 is (modulo 400)
    // 2004 is
    // 2100 is not (module 100)
    // 2400 is (modulo 400)

    if(year & 0x03)
        return 0;    // not modulo 4, not a leap year
    else
    {
        if((year % 100) != 0)
            return 1;    // modulo 4, but not modulo 100, leap year
        else if((year % 400) != 0)
            return 0;    // modul0 100, but not modulo 400, not a leap year
        else
            return 1;    // modulo 400, leap year
    }

}// isLeapYear


/*!
 * Use Gregorian date information to compute GPS style time information. The
 * output is either in GPS time or UTC time, depending on which time reference
 * is used for the inputs. This function only works for dates after Jan 1 2012
 * \param year is the Gregorian calendar year
 * \param month is the month of the year, from 1 (January) to 12 (December)
 * \param day is the day of the month, from 1 to 31
 * \param hours is the hour of the dya, from 0 to 23
 * \param minutes is the minute of the hour, from 0 to 59
 * \param seconds is the seconds of the minute, from 0 to 60 (60 may occur in a leap second jump)
 * \param milliseconds is the milliseconds of the second
 * \param pweek receives the number of weeks since Jan 6 1980 (aka GPS week number)
 * \param pitow receives the time of week in milliseconds
 */
void computeWeekAndItow(uint16_t year, uint8_t month, uint8_t day, uint8_t hours, uint8_t minutes, uint8_t seconds, int16_t milliseconds, uint16_t* pweek, uint32_t* pitow)
{
    // Number of days between Jan 6 1980 and Jan 1 2012
    uint32_t days = JAN12012;
    uint32_t week;
    uint32_t itow;

    // 0 based month
    month--;

    // 0 based day
    day--;

    // Days of the months already in this year
    days += month_day_norm[month];

    // Add in day of the month
    days += day;

    // Has leap day happened?, add a day
    if(isLeapYear(year) && (month>=2))
        days++;

    // Now add up all the years prior to this one.
    // Counting number of days to Jan 1 year 2012
    while(year > 2012)
    {
        year--;

        if(isLeapYear(year))
            days += 366;
        else
            days += 365;

    }// While still years to count down

    // Now that we have the number of days we can compute the week number
    week = days/7;

    // subtract off the days that are accounted for in the week
    days -= week*7;

    // convert the times to milliseconds
    itow = (((days*24 + hours)*60 + minutes)*60 + seconds)*1000 + milliseconds;

    // return the results
    *pweek = (uint16_t)week;
    *pitow = itow;

}// computeWeekAndItow


/*!
 * Test the date conversion logic
 * \return 1 if good, 0 if bad
 */
int testDateConversion(void)
{
    uint16_t week;
    uint32_t itow;
    uint16_t year;
    uint8_t month, day;

    // 2016 is a leap year, and march 12 is after leap day
    computeWeekAndItow(2016, 3, 12, 9, 10, 11, 250, &week, &itow);

    computeDateFromWeekAndItow(week, itow, &year, &month, &day);

    if(year != 2016)
        return 0;
    else if(month != 3)
        return 0;
    else if(day != 12)
        return 0;
    else if(week != 1887)
        return 0;
    else if(itow != (((6UL * 24UL + 9UL) * 60UL + 10UL) * 60UL + 11UL) * 1000UL + 250UL)
        return 0;
    else
        return 1;

}



