/*
 * Copyright (c) 2007, Michael Feathers, James Grenning, Bas Vodde and Timo Puronen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE EARLIER MENTIONED AUTHORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "CppUTest/TestHarness.h"

#include <e32def.h>
#include <e32std.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "CppUTest/PlatformSpecificFunctions.h"

void executePlatformSpecificTestBody(Utest* test)
{
	TInt err(KErrNone);
	TRAP(err, test->testBody());
	if(err != KErrNone) {
		Utest::getCurrent()->fail("Leave in test method", "", 0);
	}
}

void executePlatformSpecificExitCurrentTest()
{
	User::Leave(KErrNone);
}

bool executePlatformSpecificSetup(Utest* test)
{
	test->setup();
	return true;
}

void executePlatformSpecificRunOneTest(UtestShell* shell, TestPlugin* plugin, TestResult& result)
{
	shell->runOneTest(plugin, result);
}

void executePlatformSpecificTeardown(Utest* test)
{
	test->teardown();
}

static long TimeInMillisImplementation() {
	struct timeval tv;
	struct timezone tz;
	::gettimeofday(&tv, &tz);
	return (tv.tv_sec * 1000) + (long)(tv.tv_usec * 0.001);
}

static long (*timeInMillisFp) () = TimeInMillisImplementation;

long GetPlatformSpecificTimeInMillis() {
	return timeInMillisFp();
}

void SetPlatformSpecificTimeInMillisMethod(long (*platformSpecific) ()) {
	timeInMillisFp = (platformSpecific == 0) ? TimeInMillisImplementation : platformSpecific;
}

TestOutput::WorkingEnvironment PlatformSpecificGetWorkingEnvironment()
{
	return TestOutput::eclipse;
}

///////////// Time in String

static SimpleString TimeStringImplementation() {
	time_t tm = time(NULL);
	return ctime(&tm);
}

static SimpleString (*timeStringFp) () = TimeStringImplementation;

SimpleString GetPlatformSpecificTimeString() {
	return timeStringFp();
}

void SetPlatformSpecificTimeStringMethod(SimpleString (*platformMethod) ()) {
	timeStringFp = (platformMethod == 0) ? TimeStringImplementation : platformMethod;
}

int PlatformSpecificVSNprintf(char* str, unsigned int size, const char* format, va_list args) {
    return vsnprintf(str, size, format, args);
}

char PlatformSpecificToLower(char c)
{
	return tolower(c);
}

void PlatformSpecificFlush() {
	fflush(stdout);
}

int PlatformSpecificPutchar(int c) {
	return putchar(c);
}

char* PlatformSpecificStrCpy(char* s1, const char* s2) {
	return strcpy(s1, s2);
}

size_t PlatformSpecificStrLen(const char* s) {
    return strlen(s);
}

char* PlatformSpecificStrStr(const char* s1, const char* s2) {
    return strstr(s1, s2);
}

int PlatformSpecificStrCmp(const char* s1, const char* s2) {
    return strcmp(s1, s2);
}

char* PlatformSpecificStrNCpy(char* s1, const char* s2, size_t size) {
    return strncpy(s1, s2, size);
}

int PlatformSpecificStrNCmp(const char* s1, const char* s2, size_t size) {
    return strncmp(s1, s2, size);
}

char* PlatformSpecificStrCat(char* s1, const char* s2) {
    return strcat(s1, s2);
}

double PlatformSpecificFabs(double d) {
    return fabs(d);
}

void* PlatformSpecificMalloc(size_t size) {
    return malloc(size);
}

void* PlatformSpecificRealloc (void* memory, size_t size) {
    return realloc(memory, size);
}

void PlatformSpecificFree(void* memory) {
    free(memory);
}

void* PlatformSpecificMemCpy(void* s1, const void* s2, size_t size) {
    return memcpy(s1, s2, size);
}

void* PlatformSpecificMemset(void* mem, int c, size_t size)
{
	return memset(mem, c, size);
}

PlatformSpecificFile PlatformSpecificFOpen(const char* filename, const char* flag) {
    return fopen(filename, flag);
}

void PlatformSpecificFPuts(const char* str, PlatformSpecificFile file) {
    fputs(str, (FILE*)file);
}

void PlatformSpecificFClose(PlatformSpecificFile file) {
    fclose((FILE*)file);
}

int PlatformSpecificAtoI(const char*str) {
    return atoi(str);
}
