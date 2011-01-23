// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file   AP_Test.h
/// @brief  A simple unit test framework.
///
/// AP_Test provides the usual test start, condition validation and reporting
/// functions in a compact form.
///
/// Each test must be contained within a block; either a standalone function or
/// a block within a function.  The TEST macro is used to start a test; it creates
/// the local object which will track the results of the test and saves the name
/// for later reporting.  Only one test may be performed within each block.
///
/// Within the test, use the REQUIRE macro to describe a condition that must be
/// met for the test to fail.  If the condition within the macro is not met,
/// the condition will be output as a diagnostic and the test will be considered
/// to have failed.
///
/// The test ends at the end of the block, and the result of the test will be
/// output as a diagnostic.
///
/// Optionally at the end of the test suite, the Test::report method may be used
/// to summarize the results of all of the tests that were performed.
///

class Test
{
public:
    Test(const char *name);
    ~Test();
    void        require(bool expr, const char *source);
    static void report();

private:
    const char  *_name;
    bool        _fail;
    static int  _passed;
    static int  _failed;
};

Test::Test(const char *name) :
        _name(name),
        _fail(false)
{
}

Test::~Test()
{
    Serial.printf("%s: %s\n", _fail ? "FAILED" : "passed", _name);
    if (_fail) {
        _failed++;
    } else {
        _passed++;
    }
}

void
Test::require(bool expr, const char *source)
{
    if (!expr) {
        _fail = true;
        Serial.printf("%s: fail: %s\n", _name, source);
    }
}

void
Test::report()
{
    Serial.printf("\n%d passed  %d failed\n", _passed, _failed);
}

int Test::_passed = 0;
int Test::_failed = 0;

#define TEST(name)      Test _test(#name)
#define REQUIRE(expr)   _test.require(expr, #expr)

