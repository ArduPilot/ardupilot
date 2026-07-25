#include "Platform.h"
#include <stdlib.h>
#include "CppUTest/TestHarness.h"
#undef malloc
#undef free
#undef calloc
#undef realloc

#include "CppUTest/TestRegistry.h"
#include <stdio.h>
#include <stdarg.h>
#include <setjmp.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "CppUTest/PlatformSpecificFunctions.h"

#include <windows.h>
#include <mmsystem.h>


#if 0 //from GCC
static jmp_buf test_exit_jmp_buf[10];
static int jmp_buf_index = 0;

bool Utest::executePlatformSpecificSetup()
{
   if (0 == setjmp(test_exit_jmp_buf[jmp_buf_index])) {
      jmp_buf_index++;
      setup();
      jmp_buf_index--;
      return true;
   }
   return false;
}

void Utest::executePlatformSpecificTestBody()
{
   if (0 == setjmp(test_exit_jmp_buf[jmp_buf_index])) {
      jmp_buf_index++;
      testBody();
      jmp_buf_index--;
   }
}

void Utest::executePlatformSpecificTeardown()
{
   if (0 == setjmp(test_exit_jmp_buf[jmp_buf_index])) {
      jmp_buf_index++;
      teardown();
      jmp_buf_index--;
   }
}

void Utest::executePlatformSpecificRunOneTest(TestPlugin* plugin_, TestResult& result_)
{
    if (0 == setjmp(test_exit_jmp_buf[jmp_buf_index])) {
       jmp_buf_index++;
       runOneTest(plugin_, result_);
       jmp_buf_index--;
    }
}

void Utest::executePlatformSpecificExitCurrentTest()
{
   jmp_buf_index--;
   longjmp(test_exit_jmp_buf[jmp_buf_index], 1);
}

#endif

TestOutput::WorkingEnvironment PlatformSpecificGetWorkingEnvironment()
{
	return TestOutput::vistualStudio;
}

///////////// Time in millis

static long TimeInMillisImplementation()
{
    return timeGetTime()/1000;
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

static const char* TimeStringImplementation()
{
	return "Windows time needs work";
}

static const char* (*timeStringFp) () = TimeStringImplementation;

const char* GetPlatformSpecificTimeString()
{
	return timeStringFp();
}

void SetPlatformSpecificTimeStringMethod(const char* (*platformMethod) ())
{
	timeStringFp = (platformMethod == 0) ? TimeStringImplementation : platformMethod;
}


////// taken from gcc

int PlatformSpecificAtoI(const char*str)
{
   return atoi(str);
}

size_t PlatformSpecificStrLen(const char* str)
{
   return strlen(str);
}

char* PlatformSpecificStrCat(char* s1, const char* s2)
{
   return strcat(s1, s2);
}

char* PlatformSpecificStrCpy(char* s1, const char* s2)
{
   return strcpy(s1, s2);
}

char* PlatformSpecificStrNCpy(char* s1, const char* s2, size_t size)
{
   return strncpy(s1, s2, size);
}

int PlatformSpecificStrCmp(const char* s1, const char* s2)
{
   return strcmp(s1, s2);
}

int PlatformSpecificStrNCmp(const char* s1, const char* s2, size_t size)
{
   return strncmp(s1, s2, size);
}
char* PlatformSpecificStrStr(const char* s1, const char* s2)
{
   return (char*) strstr(s1, s2);
}

int PlatformSpecificVSNprintf(char *str, unsigned int size, const char* format, va_list args)
{
	char* buf = 0;
	int sizeGuess = size;

	int result = _vsnprintf( str, size, format, args);
	str[size-1] = 0;
	while (result == -1)
	{
		if (buf != 0)
			free(buf);
		sizeGuess += 10;
		buf = (char*)malloc(sizeGuess);
		result = _vsnprintf( buf, sizeGuess, format, args);
	}
	
	if (buf != 0)
		free(buf);
	return result;

}

PlatformSpecificFile PlatformSpecificFOpen(const char* filename, const char* flag)
{
   return fopen(filename, flag);
}

void PlatformSpecificFPuts(const char* str, PlatformSpecificFile file)
{
   fputs(str, (FILE*)file);
}

void PlatformSpecificFClose(PlatformSpecificFile file)
{
   fclose((FILE*)file);
}

void PlatformSpecificFlush()
{
  fflush(stdout);
}

int PlatformSpecificPutchar(int c)
{
  return putchar(c);
}

void* PlatformSpecificMalloc(size_t size)
{
   return malloc(size);
}

void* PlatformSpecificRealloc (void* memory, size_t size)
{
   return realloc(memory, size);
}

void PlatformSpecificFree(void* memory)
{
   free(memory);
}

void* PlatformSpecificMemCpy(void* s1, const void* s2, size_t size)
{
   return memcpy(s1, s2, size);
}

void* PlatformSpecificMemset(void* mem, int c, size_t size)
{
	return memset(mem, c, size);
}

double PlatformSpecificFabs(double d)
{
   return fabs(d);
}

int PlatformSpecificIsNan(double d)
{
	return _isnan(d);
}




/////// clean up the rest

#if 0

void TestRegistry::platformSpecificRunOneTest(Utest* test, TestResult& result_)
{
    try {
        runOneTest(test, result_) ;
    }
    catch (int) {
        //exiting test early
    }

}

void Utest::executePlatformSpecificTestBody()
{
	testBody();
}

void PlatformSpecificExitCurrentTestImpl()
{
    throw(1);
}

#endif

int PlatformSpecificVSNprintf(char *str, unsigned int size, const char* format, void* args)
{
   return _vsnprintf( str, size, format, (va_list) args);
}

char PlatformSpecificToLower(char c)
{
	return tolower(c);
}

//platform specific test running stuff
#if 1
#include <setjmp.h>

static jmp_buf test_exit_jmp_buf[10];
static int jmp_buf_index = 0;

bool executePlatformSpecificSetup(Utest* test)
{
   if (0 == setjmp(test_exit_jmp_buf[jmp_buf_index])) {
      jmp_buf_index++;
      test->setup();
      jmp_buf_index--;
      return true;
   }
   return false;
}

void executePlatformSpecificTestBody(Utest* test)
{
   if (0 == setjmp(test_exit_jmp_buf[jmp_buf_index])) {
      jmp_buf_index++;
      test->testBody();
      jmp_buf_index--;
   }
}

void executePlatformSpecificTeardown(Utest* test)
{
   if (0 == setjmp(test_exit_jmp_buf[jmp_buf_index])) {
      jmp_buf_index++;
      test->teardown();
      jmp_buf_index--;
   }
}

void executePlatformSpecificRunOneTest(UtestShell* shell, TestPlugin* plugin, TestResult& result)
{
    if (0 == setjmp(test_exit_jmp_buf[jmp_buf_index])) {
       jmp_buf_index++;
       shell->runOneTest(plugin, result);
       jmp_buf_index--;
    }
}


void executePlatformSpecificExitCurrentTest()
{
   jmp_buf_index--;
   longjmp(test_exit_jmp_buf[jmp_buf_index], 1);
}

/*
void PlatformSpecificExitCurrentTestImpl()
{
   jmp_buf_index--;
   longjmp(test_exit_jmp_buf[jmp_buf_index], 1);
}
*/
#endif

//static jmp_buf test_exit_jmp_buf[10];
//static int jmp_buf_index = 0;


#if 0
bool Utest::executePlatformSpecificSetup()
{
	try {
      setup();
	}
	catch (int) {
		return false;
	}
    return true;
}

void Utest::executePlatformSpecificTestBody()
{
	try {
      testBody();
	}
	catch (int) {
	}

}

void Utest::executePlatformSpecificTeardown()
{
	try {
      teardown();
	}
	catch (int) {
	}

}

void PlatformSpecificExitCurrentTestImpl()
{
	throw(1);
}


void (*PlatformSpecificExitCurrentTest)() = PlatformSpecificExitCurrentTestImpl;

void FakePlatformSpecificExitCurrentTest()
{
}

#endif

