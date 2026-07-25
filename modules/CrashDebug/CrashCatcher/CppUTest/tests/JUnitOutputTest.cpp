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

#include "CppUTest/TestHarness.h"
#include "CppUTest/JUnitTestOutput.h"
#include "CppUTest/TestResult.h"
#include "CppUTest/PlatformSpecificFunctions.h"

static long millisTime;

static const char* theTime = "1978-10-03T00:00:00";

static long MockGetPlatformSpecificTimeInMillis()
{
	return millisTime;
}

static const char* MockGetPlatformSpecificTimeString()
{
	return theTime;
}

TEST_GROUP(JUnitOutputTest)
{
	class MockJUnitTestOutput: public JUnitTestOutput
	{
	public:
		enum
		{
			testGroupSize = 10
		};
		enum
		{
			defaultSize = 7
		};

		int filesOpened;
		int fileBalance;

		SimpleString fileName_;
		SimpleString buffer_;

		TestResult* res_;
		struct TestData
		{
			TestData() :
				tst_(0), testName_(0), failure_(0)
			{
			}
			;
			UtestShell* tst_;
			SimpleString* testName_;
			TestFailure* failure_;
		};

		struct TestGroupData
		{
			TestGroupData() :
				numberTests_(0), totalFailures_(0), name_(""), testData_(0)
			{
			}
			;

			int numberTests_;
			int totalFailures_;
			SimpleString name_;

			TestData* testData_;
		};

		TestGroupData testGroupData_[testGroupSize];

		TestGroupData& currentGroup()
		{
			return testGroupData_[filesOpened - 1];
		}

		void resetXmlFile()
		{
			buffer_ = "";
		}

		MockJUnitTestOutput() :
			filesOpened(0), fileBalance(0), res_(0)
		{
			for (int i = 0; i < testGroupSize; i++) {
				testGroupData_[i].numberTests_ = 0;
				testGroupData_[i].totalFailures_ = 0;
			}
		}
		;

		void setResult(TestResult* testRes)
		{
			res_ = testRes;
		}

		virtual ~MockJUnitTestOutput()
		{
			for (int i = 0; i < testGroupSize; i++) {
				for (int j = 0; j < testGroupData_[i].numberTests_; j++) {
					delete testGroupData_[i].testData_[j].tst_;
					delete testGroupData_[i].testData_[j].testName_;
					if (testGroupData_[i].testData_[j].failure_) delete testGroupData_[i].testData_[j].failure_;
				}
				if (testGroupData_[i].testData_) delete[] testGroupData_[i].testData_;
			}

			LONGS_EQUAL(0, fileBalance);
		}

		void writeToFile(const SimpleString& buf)
		{
			buffer_ += buf;
		}

		void openFileForWrite(const SimpleString& in_FileName)
		{
			filesOpened++;
			fileBalance++;
			fileName_ = in_FileName;
		}

		void closeFile()
		{
			CHECK_XML_FILE();
			resetXmlFile();
			fileBalance--;
		}

		void createTestsInGroup(int index, int amount, const char* group, const char* basename)
		{
			testGroupData_[index].name_ = group;
			testGroupData_[index].numberTests_ = amount;

			testGroupData_[index].testData_ = new TestData[amount];
			for (int i = 0; i < amount; i++) {
				TestData& testData = testGroupData_[index].testData_[i];
				testData.testName_ = new SimpleString(basename);
				*testData.testName_ += StringFrom((long) i);
				testData.tst_ = new UtestShell(group, testData.testName_->asCharString(), "file", 1);
			}
		}
		void runTests()
		{
			res_->testsStarted();
			for (int i = 0; i < testGroupSize; i++) {
				TestGroupData& data = testGroupData_[i];
				if (data.numberTests_ == 0) continue;

				millisTime = 0;
				res_->currentGroupStarted(data.testData_[0].tst_);
				for (int j = 0; j < data.numberTests_; j++) {
					TestData& testData = data.testData_[j];

					millisTime = 0;
					res_->currentTestStarted(testData.tst_);
					if (testData.failure_) print(*testData.failure_);
					millisTime = 10;
					res_->currentTestEnded(testData.tst_);
				}
				millisTime = 50;
				res_->currentGroupEnded(data.testData_[0].tst_);
			}
			res_->testsEnded();
		}

		void setFailure(int groupIndex, int testIndex, const char* fileName, int lineNumber, const char* message)
		{
			TestData& data = testGroupData_[groupIndex].testData_[testIndex];
			data.failure_ = new TestFailure(data.tst_, fileName, lineNumber, message);
			testGroupData_[groupIndex].totalFailures_++;
		}

		void CHECK_HAS_XML_HEADER(SimpleString string)
		{
			STRCMP_EQUAL("<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n", string.asCharString());
		}

		void CHECK_TEST_SUITE_START(SimpleString output)
		{
			TestGroupData& group = currentGroup();
			SimpleString buf = StringFromFormat("<testsuite errors=\"0\" failures=\"%d\" hostname=\"localhost\" name=\"%s\" tests=\"%d\" time=\"0.050\" timestamp=\"%s\">\n", group.totalFailures_,
					group.name_.asCharString(), group.numberTests_, theTime);
			CHECK_EQUAL(buf, output);
		}

		void CHECK_XML_FILE()
		{
			size_t totalSize = currentGroup().numberTests_ + defaultSize + (currentGroup().totalFailures_ * 2);
			SimpleStringCollection col;
			buffer_.split("\n", col);
			CHECK(col.size() >= totalSize);
			CHECK_HAS_XML_HEADER(col[0]);
			CHECK_TEST_SUITE_START(col[1]);
			CHECK_PROPERTIES_START(col[2]);
			CHECK_PROPERTIES_END(col[3]);
			CHECK_TESTS(&col[4]);
			CHECK_SYSTEM_OUT(col[col.size() - 3]);
			CHECK_SYSTEM_ERR(col[col.size() - 2]);
			CHECK_TEST_SUITE_END(col[col.size() - 1]);
		}

		void CHECK_PROPERTIES_START(const SimpleString& output)
		{
			STRCMP_EQUAL("<properties>\n", output.asCharString());
		}

		void CHECK_PROPERTIES_END(const SimpleString& output)
		{
			STRCMP_EQUAL("</properties>\n", output.asCharString());
		}

		void CHECK_SYSTEM_OUT(const SimpleString& output)
		{
			STRCMP_EQUAL("<system-out></system-out>\n", output.asCharString());
		}

		void CHECK_SYSTEM_ERR(const SimpleString& output)
		{
			STRCMP_EQUAL("<system-err></system-err>\n", output.asCharString());
		}

		void CHECK_TEST_SUITE_END(const SimpleString& output)
		{
			STRCMP_EQUAL("</testsuite>", output.asCharString());
		}
		void CHECK_TESTS(SimpleString* arr)
		{
			for (int index = 0, curTest = 0; curTest < currentGroup().numberTests_; curTest++, index++) {
				SimpleString buf = StringFromFormat("<testcase classname=\"%s\" name=\"%s\" time=\"0.010\">\n", currentGroup().name_.asCharString(),
						currentGroup().testData_[curTest].tst_->getName().asCharString());
				CHECK_EQUAL(buf, arr[index]);
				if (currentGroup().testData_[curTest].failure_) {
					CHECK_FAILURE(arr, index, curTest);
				}
				buf = "</testcase>\n";
				CHECK_EQUAL(buf, arr[++index]);

			}
		}
		void CHECK_FAILURE(SimpleString* arr, int& i, int curTest)
		{
			TestFailure& f = *currentGroup().testData_[curTest].failure_;
			i++;
			SimpleString message = f.getMessage().asCharString();
			message.replace('"', '\'');
			message.replace('<', '[');
			message.replace('>', ']');
			message.replace("\n", "{newline}");
			SimpleString buf = StringFromFormat("<failure message=\"%s:%d: %s\" type=\"AssertionFailedError\">\n", f.getFileName().asCharString(), f.getFailureLineNumber(), message.asCharString());
			CHECK_EQUAL(buf, arr[i]);
			i++;
			STRCMP_EQUAL("</failure>\n", arr[i].asCharString());
		}
	};

	MockJUnitTestOutput * output;
	TestResult *res;

	void setup()
	{
		output = new MockJUnitTestOutput();
		res = new TestResult(*output);
		output->setResult(res);
		SetPlatformSpecificTimeInMillisMethod(MockGetPlatformSpecificTimeInMillis);
		SetPlatformSpecificTimeStringMethod(MockGetPlatformSpecificTimeString);
	}
	void teardown()
	{
		delete output;
		delete res;
		SetPlatformSpecificTimeInMillisMethod(0);
		SetPlatformSpecificTimeStringMethod(0);
	}

	void runTests()
	{
		output->printTestsStarted();
		output->runTests();
		output->printTestsEnded(*res);
	}
};

TEST(JUnitOutputTest, oneTestInOneGroupAllPass)
{
	output->createTestsInGroup(0, 1, "group", "name");
	runTests();
	STRCMP_EQUAL("cpputest_group.xml", output->fileName_.asCharString());
	LONGS_EQUAL(1, output->filesOpened);
}

TEST(JUnitOutputTest, fiveTestsInOneGroupAllPass)
{
	output->createTestsInGroup(0, 5, "group", "name");
	runTests();
}

TEST(JUnitOutputTest, multipleTestsInTwoGroupAllPass)
{
	output->createTestsInGroup(0, 3, "group", "name");
	output->createTestsInGroup(1, 8, "secondGroup", "secondName");
	runTests();
	LONGS_EQUAL(2, output->filesOpened);
}

TEST(JUnitOutputTest, oneTestInOneGroupFailed)
{
	output->createTestsInGroup(0, 1, "failedGroup", "failedName");
	output->setFailure(0, 0, "file", 1, "Test <\"just\"> failed");
	runTests();
}

TEST(JUnitOutputTest, fiveTestsInOneGroupAndThreeFail)
{
	output->printTestsStarted();
	output->createTestsInGroup(0, 5, "failedGroup", "failedName");
	output->setFailure(0, 0, "file", 1, "Test just failed");
	output->setFailure(0, 1, "file", 5, "Also failed");
	output->setFailure(0, 4, "file", 8, "And failed again");
	runTests();
}

TEST(JUnitOutputTest, fourGroupsAndSomePassAndSomeFail)
{
	output->printTestsStarted();
	output->createTestsInGroup(0, 5, "group1", "firstName");
	output->createTestsInGroup(1, 50, "group2", "secondName");
	output->createTestsInGroup(2, 3, "group3", "thirdName");
	output->createTestsInGroup(3, 5, "group4", "fourthName");

	output->setFailure(0, 0, "file", 1, "Test just failed");
	output->printTestsEnded(*res);
	runTests();
}

TEST(JUnitOutputTest, messageWithNewLine)
{
	output->createTestsInGroup(0, 1, "failedGroup", "failedName");
	output->setFailure(0, 0, "file", 1, "Test \n failed");
	runTests();
}

TEST(JUnitOutputTest, createNormalFilename)
{
	STRCMP_EQUAL("cpputest_group.xml", output->createFileName("group").asCharString());
}

TEST(JUnitOutputTest, escapeSlashesInFilenames)
{
	STRCMP_EQUAL("cpputest_group_weird_name.xml", output->createFileName("group/weird/name").asCharString());
}


