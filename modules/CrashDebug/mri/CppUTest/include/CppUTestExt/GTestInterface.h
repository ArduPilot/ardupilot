/*
 * Copyright (c) 2011, Michael Feathers, James Grenning and Bas Vodde
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

#include "CppUTest/Utest.h"
#include "CppUTest/TestResult.h"
#include "CppUTest/TestFailure.h"

#define TEST(testGroup, testName) \
  class TEST_##testGroup##_##testName##_Test : public Utest \
{ public: TEST_##testGroup##_##testName##_Test () : Utest () {} \
       void testBody(); }; \
  class TEST_##testGroup##_##testName##_TestShell : public UtestShell \
{  public: virtual Utest* createTest() { return new TEST_##testGroup##_##testName##_Test; } \
  } TEST_##testGroup##_##testName##_TestShell_Instance; \
  TestInstaller TEST_##testGroup##_##testName##_Installer(TEST_##testGroup##_##testName##_TestShell_Instance, #testGroup, #testName, __FILE__,__LINE__); \
	void TEST_##testGroup##_##testName##_Test::testBody()

#define TEST_F(testGroup, testName) \
  class TEST_##testGroup##_##testName##_Test : public testGroup \
{ public: TEST_##testGroup##_##testName##_Test () : testGroup () {} \
       void testBody(); }; \
  class TEST_##testGroup##_##testName##_TestShell : public UtestShell { \
	  virtual Utest* createTest() { return new TEST_##testGroup##_##testName##_Test; } \
  } TEST_##testGroup##_##testName##_TestShell_instance; \
  TestInstaller TEST_##testGroup##_##testName##_Installer(TEST_##testGroup##_##testName##_TestShell_instance, #testGroup, #testName, __FILE__,__LINE__); \
	void TEST_##testGroup##_##testName##_Test::testBody()

/*
 * NOTICE:
 *
 * Code duplicated from UtestMacros.h. Its hard to share as don't want to include the CppUTest
 * macros in the gtest interface.
 *
 */

#define EXPECT_EQ(expected, actual) \
  if ((expected) != (actual))\
  {\
	 { \
      UtestShell::getTestResult()->countCheck();\
  	   CheckEqualFailure _f(UtestShell::getCurrent(), __FILE__, __LINE__, StringFrom(expected), StringFrom(actual)); \
      UtestShell::getTestResult()->addFailure(_f);\
    } \
    UtestShell::getCurrent()->exitCurrentTest(); \
  }\
  else\
	 UtestShell::getTestResult()->countCheck();

#define EXPECT_TRUE(condition) \
	{ UtestShell::getCurrent()->assertTrue((condition) != 0, "EXPECT_TRUE", #condition, __FILE__, __LINE__); }

#define EXPECT_FALSE(condition) \
	{ UtestShell::getCurrent()->assertTrue((condition) == 0, "EXPECT_FALSE", #condition, __FILE__, __LINE__); }

#define EXPECT_STREQ(expected, actual) \
	{ UtestShell::getCurrent()->assertCstrEqual(expected, actual, __FILE__, __LINE__); }

#define ASSERT_EQ(expected, actual) EXPECT_EQ(expected, actual)

#define ASSERT_TRUE(condition) EXPECT_TRUE(condition)

namespace testing
{
	class Test : public Utest
	{
		virtual void SetUp(){}
		virtual void TearDown(){}

		void setup()
		{
			SetUp();
		}

		void teardown()
		{
			TearDown();
		}

	};
}



