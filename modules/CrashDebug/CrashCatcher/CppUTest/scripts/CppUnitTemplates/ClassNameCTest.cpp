#include "CppUTest/TestHarness.h"

extern "C"
{
#include "ClassName.h"
}

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

