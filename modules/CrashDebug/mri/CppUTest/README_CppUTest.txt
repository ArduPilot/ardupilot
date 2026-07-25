The unit test harness supplied with CppUTest is based on Michael 
Feathers' CppUnitLite.

CppUTest.UnitTestHarness supports these features

Command line switches

    * -v verbose, print each test name as it runs
    * -r# repeat the tests some number of times, default is one, default 
          is # is not specified is 2. This is handy if you are experiencing 
          memory leaks. A second run that has no leaks indicates that someone 
          is allocating statics and not releasing them.
    * -g group only run test whose group contains the substring group
    * -n name only run test whose name contains the substring name


Test Macros

    * TEST(group, name) - define a test
    * IGNORE_TEST(group, name) - turn off the execution of a test
    * TEST_GROUP(group) - Declare a test group to which certain tests belong.
                          This will also create thelink needed from another library.
    * TEST_GROUP_BASE(group, base) - Same as TEST_GROUP, just use a different 
                          base class than Utest
    * TEST_SETUP() - Declare a void setup method in a TEST_GROUP
            - this is the same as declaring void setup()
    * TEST_TEARDOWN() - Declare a void setup method in a TEST_GROUP
    * IMPORT_TEST_GROUP(group) - Export the name of a test group so it can 
      be linked in from a library. Needs to be done in main.


Set up and tear down support

    * Each TEST_GROUP may contain a setup and/or  a teardown method.
    * setup() is called prior to each TEST body and teardown() is 
      called after the test body.


Assertion Macros
The failure of one of these macros causes the current test to immediately exit

    * CHECK(boolean condition) - checks any boolean result
    * CHECK_TRUE(boolean condition) - checks for true
    * CHECK_FALSE(boolean condition) - checks for false
    * CHECK_EQUAL(expected, actual) - checks for equality between entities 
      using ==. So if you have a class that supports operator==() you can use 
      this macro to compare two instances.
    * STRCMP_EQUAL(expected, actual) - check const char* strings for equality 
      using strcmp
    * LONGS_EQUAL(expected, actual) - Compares two numbers
    * BYTES_EQUAL(expected, actual) - Compares two numbers, eight bits wide
    * POINTERS_EQUAL(expected, actual) - Compares two const void * 
    * DOUBLES_EQUAL(expected, actual, tolerance) - Compares two doubles 
      within some tolerance
    * FAIL(text) - always fails


Customize CHECK_EQUAL to work with your types that support operator==()

    * Create the function

        SimpleString StringFrom (const yourType&)
        
    The Extensions directory has a few of these.

Building default checks with TestPlugin

    * CppUTest can support extra checking functionality by 
      inserting TestPlugins
    * TestPlugin is derived from the TestPlugin class and can be 
      inserted in the TestRegistry via the installPlugin method.
    * All TestPlugins are called before and after running all tests 
      and before and after running a single test (like Setup and 
      Teardown). TestPlugins are typically inserted in the main.
    * TestPlugins can be used for, for example, system stability 
      and resource handling like files, memory or network connection 
      clean-up.
    * In CppUTest, the memory leak detection is done via a default 
      enabled TestPlugin

Example of a main with a TestPlugin:

int main(int ac, char** av)
{
   LogPlugin logPlugin;
   TestRegistry::getCurrentRegistry()->installPlugin(&logPlugin);
   int result = CommandLineTestRunner::RunAllTests(ac, av);
   TestRegistry::getCurrentRegistry()->resetPlugins();
   return result;
}

Memory leak detection

    * A platform specific memory leak detection mechanism is provided.
    * If a test fails and has allocated memory prior to the fail and 
      that memory is not cleaned up by TearDown, a memory leak is reported. 
      It is best to only chase memory leaks when other errors have 
      been eliminated.
    * Some code uses lazy initialization and appears to leak when it 
      really does not (for example: gcc stringstream used to in an 
      earlier release). One cause is that some standard library calls 
      allocate something and do not free it until after main (or never). 
      To find out if a memory leak is due to lazy initialization set 
      the -r switch to run tests twice. The signature of this situation 
      is that the first run shows leaks and the second run shows no 
      leaks. When both runs show leaks, you have a leak to find.

How is memory leak detection implemented?

    * Before SetUp() a memory usage checkpoint is recorded
    * After TearDown() another checkpoint is taken and compared to the 
      original checkpoint
    * In Visual Studio the MS debug heap capabilities are used
    * For GCC a simple new/delete count is used in overridden operators new,  
      new[], delete and delete[]


If you use some leaky code that you can't or won't fix you can tell a 
TEST to ignore a certain number of leaks as in this example:

TEST(MemoryLeakWarningTest, Ignore1)
{
    EXPECT_N_LEAKS(1);
    char* arrayToLeak1 = new char[100];
}


Example Main

#include "UnitTestHarness/CommandLineTestRunner.h"

int main(int ac, char** av)
{
  return CommandLineTestRunner::RunAllTests(ac, av);
}

IMPORT_TEST_GROUP(ClassName)


Example Test

#include "UnitTestHarness/TestHarness.h"
#include "ClassName.h"

TEST_GROUP(ClassName)
{
  ClassName* className;

  void setup()
  {
    className = new ClassName();
  }
  void teardown()
  {
    delete className;
  }
}

TEST(ClassName, Create)
{
  CHECK(0 != className);
  CHECK(true);
  CHECK_EQUALS(1,1);
  LONGS_EQUAL(1,1);
  DOUBLES_EQUAL(1.000, 1.001, .01);
  STRCMP_EQUAL("hello", "hello");
  FAIL("The prior tests pass, but this one doesn't");
}

There are some scripts that are helpful in creating your initial h, cpp, and 
Test files.  See scripts/README.TXT

