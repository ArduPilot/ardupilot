extern "C"
{
#include "ClassName.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(ClassName)
{
    void setup()
    {
      ClassName_Create();
    }

    void teardown()
    {
       ClassName_Destroy();
    }
};

TEST(ClassName, Create)
{
  FAIL("Start here");
}

