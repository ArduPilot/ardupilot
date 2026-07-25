/*
 * Copyright (c) 2007, Michael Feathers, James Grenning and Bas Vodde
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

#ifndef D_UTestMacros_h
#define D_UTestMacros_h

/*! \brief Define a group of tests
 *
 * All tests in a TEST_GROUP share the same setup()
 * and teardown().  setup() is run before the opening
 * curly brace of each TEST and teardown() is
 * called after the closing curly brace of TEST.
 *
 */


#define TEST_GROUP_BASE(testGroup, baseclass) \
  int externTestGroup##testGroup = 0; \
  struct TEST_GROUP_##CppUTestGroup##testGroup : public baseclass

#define TEST_BASE(testBaseClass) \
  struct testBaseClass : public Utest

#define TEST_GROUP(testGroup) \
  TEST_GROUP_BASE(testGroup, Utest)

#define TEST_SETUP() \
  virtual void setup()

#define TEST_TEARDOWN() \
  virtual void teardown()

#define TEST(testGroup, testName) \
  class TEST_##testGroup##_##testName##_Test : public TEST_GROUP_##CppUTestGroup##testGroup \
{ public: TEST_##testGroup##_##testName##_Test () : TEST_GROUP_##CppUTestGroup##testGroup () {} \
       void testBody(); }; \
  class TEST_##testGroup##_##testName##_TestShell : public UtestShell { \
	  virtual Utest* createTest() { return new TEST_##testGroup##_##testName##_Test; } \
  } TEST_##testGroup##_##testName##_TestShell_instance; \
  TestInstaller TEST_##testGroup##_##testName##_Installer(TEST_##testGroup##_##testName##_TestShell_instance, #testGroup, #testName, __FILE__,__LINE__); \
	void TEST_##testGroup##_##testName##_Test::testBody()

#define IGNORE_TEST(testGroup, testName)\
  class IGNORE##testGroup##_##testName##_Test : public TEST_GROUP_##CppUTestGroup##testGroup \
{ public: IGNORE##testGroup##_##testName##_Test () : TEST_GROUP_##CppUTestGroup##testGroup () {} \
  public: void testBodyThatNeverRuns (); }; \
  class IGNORE##testGroup##_##testName##_TestShell : public IgnoredUtestShell { \
	  virtual Utest* createTest() { return new IGNORE##testGroup##_##testName##_Test; } \
  } IGNORE##testGroup##_##testName##_TestShell_instance; \
   TestInstaller TEST_##testGroup##testName##_Installer(IGNORE##testGroup##_##testName##_TestShell_instance, #testGroup, #testName, __FILE__,__LINE__); \
	void IGNORE##testGroup##_##testName##_Test::testBodyThatNeverRuns ()

#define IMPORT_TEST_GROUP(testGroup) \
  extern int externTestGroup##testGroup;\
  int* p##testGroup = &externTestGroup##testGroup

//Check any boolean condition

#define CHECK(condition)\
  CHECK_LOCATION_TRUE(condition, "CHECK", #condition, __FILE__, __LINE__)

#define CHECK_TRUE(condition)\
  CHECK_LOCATION_TRUE(condition, "CHECK_TRUE", #condition, __FILE__, __LINE__)

#define CHECK_FALSE(condition)\
  CHECK_LOCATION_FALSE(condition, "CHECK_FALSE", #condition, __FILE__, __LINE__)

#define CHECK_LOCATION_TRUE(condition, checkString, conditionString, file, line)\
  { UtestShell::getCurrent()->assertTrue((condition) != 0, checkString, conditionString, file, line); }

#define CHECK_LOCATION_FALSE(condition, checkString, conditionString, file, line)\
  { UtestShell::getCurrent()->assertTrue((condition) == 0, checkString, conditionString, file, line); }

//This check needs the operator!=(), and a StringFrom(YourType) function
#define CHECK_EQUAL(expected,actual)\
  CHECK_EQUAL_LOCATION(expected, actual, __FILE__, __LINE__)

#define CHECK_EQUAL_LOCATION(expected,actual, file, line)\
  if ((expected) != (actual))\
  {\
	 { \
      UtestShell::getTestResult()->countCheck();\
  	   CheckEqualFailure _f(UtestShell::getCurrent(), file, line, StringFrom(expected), StringFrom(actual)); \
      UtestShell::getTestResult()->addFailure(_f);\
    } \
    UtestShell::getCurrent()->exitCurrentTest(); \
  }\
  else\
	 UtestShell::getTestResult()->countCheck();

//This check checks for char* string equality using strcmp.
//This makes up for the fact that CHECK_EQUAL only compares the pointers to char*'s
#define STRCMP_EQUAL(expected,actual)\
  STRCMP_EQUAL_LOCATION(expected, actual, __FILE__, __LINE__)

#define STRCMP_EQUAL_LOCATION(expected,actual, file, line)\
  { UtestShell::getCurrent()->assertCstrEqual(expected, actual, file, line); }

#define STRCMP_NOCASE_EQUAL(expected,actual)\
  STRCMP_NOCASE_EQUAL_LOCATION(expected, actual, __FILE__, __LINE__)

#define STRCMP_NOCASE_EQUAL_LOCATION(expected,actual, file, line)\
  { UtestShell::getCurrent()->assertCstrNoCaseEqual(expected, actual, file, line); }

#define STRCMP_CONTAINS(expected,actual)\
  STRCMP_CONTAINS_LOCATION(expected, actual, __FILE__, __LINE__)

#define STRCMP_CONTAINS_LOCATION(expected,actual, file, line)\
  { UtestShell::getCurrent()->assertCstrContains(expected, actual, file, line); }

#define STRCMP_NOCASE_CONTAINS(expected,actual)\
  STRCMP_NOCASE_CONTAINS_LOCATION(expected, actual, __FILE__, __LINE__)

#define STRCMP_NOCASE_CONTAINS_LOCATION(expected,actual, file, line)\
  { UtestShell::getCurrent()->assertCstrNoCaseContains(expected, actual, file, line); }

//Check two long integers for equality
#define LONGS_EQUAL(expected,actual)\
  LONGS_EQUAL_LOCATION(expected,actual,__FILE__, __LINE__)

#define LONGS_EQUAL_LOCATION(expected,actual,file,line)\
  { UtestShell::getCurrent()->assertLongsEqual((long)expected, (long)actual,  file, line); }

#define BYTES_EQUAL(expected, actual)\
    LONGS_EQUAL((expected) & 0xff,(actual) & 0xff)

#define POINTERS_EQUAL(expected, actual)\
    POINTERS_EQUAL_LOCATION((expected),(actual), __FILE__, __LINE__)

#define POINTERS_EQUAL_LOCATION(expected,actual,file,line)\
  { UtestShell::getCurrent()->assertPointersEqual((void *)expected, (void *)actual,  file, line); }

//Check two doubles for equality within a tolerance threshold
#define DOUBLES_EQUAL(expected,actual,threshold)\
  DOUBLES_EQUAL_LOCATION(expected,actual,threshold,__FILE__,__LINE__)

#define DOUBLES_EQUAL_LOCATION(expected,actual,threshold,file,line)\
  { UtestShell::getCurrent()->assertDoublesEqual(expected, actual, threshold,  file, line); }

//Fail if you get to this macro
//The macro FAIL may already be taken, so allow FAIL_TEST too
#ifndef FAIL
#define FAIL(text)\
  FAIL_LOCATION(text, __FILE__,__LINE__)

#define FAIL_LOCATION(text, file, line)\
  { UtestShell::getCurrent()->fail(text,  file, line); UtestShell::getCurrent()->exitCurrentTest(); }
#endif

#define FAIL_TEST(text)\
  FAIL_TEST_LOCATION(text, __FILE__,__LINE__)

#define FAIL_TEST_LOCATION(text, file,line)\
  { UtestShell::getCurrent()->fail(text, file, line); UtestShell::getCurrent()->exitCurrentTest(); }

#define UT_PRINT_LOCATION(text, file, line) \
   { UtestShell::getCurrent()->print(text, file, line); }

#define UT_PRINT(text) \
   UT_PRINT_LOCATION(text, __FILE__, __LINE__)

#define UT_CRASH() { UtestShell::crash(); }
#define RUN_ALL_TESTS(ac, av) CommandLineTestRunner::RunAllTests(ac, av)

#endif /*D_UTestMacros_h*/
