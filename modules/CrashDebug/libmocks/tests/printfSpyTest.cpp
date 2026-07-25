/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include <string.h>
// Include headers from C modules under test.
extern "C"
{
    #include "printfSpy.h"
}

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


const char g_HelloWorld[] = "Hello World!\n";


TEST_GROUP(printfSpy)
{
    int  m_Result;
    char m_checkBuffer[32];

    void setup()
    {
        m_Result = -1;
    }

    void teardown()
    {
        printfSpy_Unhook();
    }

    void printfCheck(int ExpectedLength, const char* pExpectedString)
    {
        LONGS_EQUAL(ExpectedLength, m_Result);
        STRCMP_EQUAL(pExpectedString, printfSpy_GetLastOutput());
        POINTERS_EQUAL(NULL, printfSpy_GetPreviousOutput());
        LONGS_EQUAL(1, printfSpy_GetCallCount());
    }

    void CreateCheckBuffer(size_t BufferSize)
    {
        CHECK ( BufferSize < sizeof(m_checkBuffer) );
        strncpy(m_checkBuffer, g_HelloWorld, sizeof(m_checkBuffer));
        m_checkBuffer[BufferSize] = '\0';
    }

    void printfCheckHelloWorldWithBufferOfSize(size_t BufferSize)
    {
        printfSpy_Hook(BufferSize);
        m_Result = hook_printf(g_HelloWorld);

        CreateCheckBuffer(BufferSize);
        printfCheck(sizeof(g_HelloWorld)-1, m_checkBuffer);
    }
};


TEST(printfSpy, BufferSize0)
{
    printfCheckHelloWorldWithBufferOfSize(0);
}


TEST(printfSpy, BufferSize1)
{
    printfCheckHelloWorldWithBufferOfSize(1);
}

TEST(printfSpy, BufferSizeMinus1)
{
    printfCheckHelloWorldWithBufferOfSize(sizeof(g_HelloWorld)-2);
}

TEST(printfSpy, BufferSizeExact)
{
    printfCheckHelloWorldWithBufferOfSize(sizeof(g_HelloWorld)-1);
}

TEST(printfSpy, BufferSizePlus1)
{
    printfCheckHelloWorldWithBufferOfSize(sizeof(g_HelloWorld));
}

TEST(printfSpy, WithFormatting)
{
    printfSpy_Hook(10);
    m_Result = hook_printf("Hello %s\n", "World");

    printfCheck(12, "Hello Worl");
}

TEST(printfSpy, TwoCall)
{
    printfSpy_Hook(10);
    hook_printf("Line 1\r\n");
    hook_printf("Line 2\r\n");
    LONGS_EQUAL(2, printfSpy_GetCallCount());
    STRCMP_EQUAL("Line 2\r\n", printfSpy_GetLastOutput());
    STRCMP_EQUAL("Line 1\r\n", printfSpy_GetPreviousOutput());
}

TEST(printfSpy, SendFourLinesToStdOut)
{
    printfSpy_Hook(10);
    hook_printf("Line 1\r\n");
    hook_printf("Line 2\r\n");
    hook_printf("Line 3\r\n");
    hook_printf("Line 4\r\n");
    STRCMP_EQUAL("Line 4\r\n", printfSpy_GetNthOutput(1));
    STRCMP_EQUAL("Line 3\r\n", printfSpy_GetNthOutput(2));
    STRCMP_EQUAL("Line 2\r\n", printfSpy_GetNthOutput(3));
    STRCMP_EQUAL("Line 1\r\n", printfSpy_GetNthOutput(4));
}

TEST(printfSpy, SendOneLineToStdErr)
{
    printfSpy_Hook(10);
    hook_fprintf(stderr, "Line 1\r\n");
    POINTERS_EQUAL(NULL, printfSpy_GetLastOutput());
    STRCMP_EQUAL("Line 1\r\n", printfSpy_GetLastErrorOutput());
    POINTERS_EQUAL(NULL, printfSpy_GetPreviousErrorOutput());
}

TEST(printfSpy, SendTwoLinesToStdErr)
{
    printfSpy_Hook(10);
    hook_fprintf(stderr, "Line 1\r\n");
    hook_fprintf(stderr, "Line 2\r\n");
    POINTERS_EQUAL(NULL, printfSpy_GetLastOutput());
    POINTERS_EQUAL(NULL, printfSpy_GetPreviousOutput());
    STRCMP_EQUAL("Line 2\r\n", printfSpy_GetLastErrorOutput());
    STRCMP_EQUAL("Line 1\r\n", printfSpy_GetPreviousErrorOutput());
}

TEST(printfSpy, SendFourLinesToStdErr)
{
    printfSpy_Hook(10);
    hook_fprintf(stderr, "Line 1\r\n");
    hook_fprintf(stderr, "Line 2\r\n");
    hook_fprintf(stderr, "Line 3\r\n");
    hook_fprintf(stderr, "Line 4\r\n");
    STRCMP_EQUAL("Line 4\r\n", printfSpy_GetNthErrorOutput(1));
    STRCMP_EQUAL("Line 3\r\n", printfSpy_GetNthErrorOutput(2));
    STRCMP_EQUAL("Line 2\r\n", printfSpy_GetNthErrorOutput(3));
    STRCMP_EQUAL("Line 1\r\n", printfSpy_GetNthErrorOutput(4));
}
