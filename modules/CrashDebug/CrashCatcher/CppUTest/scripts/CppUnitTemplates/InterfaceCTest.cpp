#include "CppUTest/TestHarness.h"

extern "C"
{
#include "FakeClassName.h"
}

TEST_GROUP(FakeClassName)
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

TEST(FakeClassName, Create)
{
  FAIL("Start here");
}
