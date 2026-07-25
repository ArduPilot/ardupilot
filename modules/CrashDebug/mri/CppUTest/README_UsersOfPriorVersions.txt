If you were a user of CppTestTools you will have a few changes to make.

CppUTest is the unit test harness from CppTestTools
CppFit is the FIT implementaions from CppTestTools 
	(CppFit is a separate download)

Sorry, this is not a complete set of instructions for converting, but
here are some suggestions.

In each test file
	change namespace for SetUp and TearDown to TEST_GROUP(GroupName)
		(GroupName is the class name by convention)
	delete IMPORT_TEST_GROUP
		(TEST_GROUP has this built in now)
	#include "UnitTestHarness/somefile.h" should be 
		#include "CppUTest/somefile.h"
		
Your Makefiles have to change:
	Change DOTO to OBJS
	Replace CPP_TEST_TOOLS with CPP_U_TEST
	Replace MakefileHelpers with build
	Change -I$(CPP_TEST_TOOLS) to -I$(CPP_U_TEST)/include\
	For libraries using fixtures add -I$(CPP_FIT)/include
	Add the RunAllTests.sh script to your AllTests directory


	