
#include "CppUTest/TestHarness.h"
#include "CppUTest/TestResult.h"
#include <time.h>
#include <sys/time.h>

void executePlatformSpecificTestBody(Utest* test)
{
	test->testBody();
}


///////////// Time in millis

static long TimeInMillisImplementation()
{
	struct timeval tv;
	struct timezone tz;
	::gettimeofday(&tv, &tz);
	return (tv.tv_sec * 1000) + (long)(tv.tv_usec * 0.001);	
}

static long (*timeInMillisFp) () = TimeInMillisImplementation;

long GetPlatformSpecificTimeInMillis()
{
	return timeInMillisFp();
}

void SetPlatformSpecificTimeInMillisMethod(long (*platformSpecific) ())
{
	timeInMillisFp = (platformSpecific == 0) ? TimeInMillisImplementation : platformSpecific;
}

///////////// Time in String

static SimpleString TimeStringImplementation()
{
	time_t tm = time(NULL);
	return ctime(&tm);
}

static SimpleString (*timeStringFp) () = TimeStringImplementation;

SimpleString GetPlatformSpecificTimeString()
{
	return timeStringFp();
}

void SetPlatformSpecificTimeStringMethod(SimpleString (*platformMethod) ())
{
	timeStringFp = (platformMethod == 0) ? TimeStringImplementation : platformMethod;
}

///////////// Run one test with exit on first error, using setjmp/longjmp
#include <setjmp.h>

static jmp_buf test_exit_jmp_buf;

void TestRegistry::platformSpecificRunOneTest(Utest* test, TestResult& result)
{
    if (0 == setjmp(test_exit_jmp_buf))
        runOneTest(test, result) ;
}

void PlatformSpecificExitCurrentTestImpl() 
{
    longjmp(test_exit_jmp_buf, 1);
}

void FakePlatformSpecificExitCurrentTest() 
{
}

void (*PlatformSpecificExitCurrentTest)() = PlatformSpecificExitCurrentTestImpl;


