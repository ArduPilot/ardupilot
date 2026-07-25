#include "CppUTest/TestHarness.h"

static int fakeRan = 0;

extern "C"
{
#include "ClassName.h"
    void virtualFunction_renameThis_fake(ClassName*)
    {
        fakeRan = 1;
    }
}

TEST_GROUP(ClassName)
{
    ClassName* aClassName;

    void setup()
    {
      aClassName = ClassName_Create();
      fakeRan = 0;
      aClassName->virtualFunction_renameThis = virtualFunction_renameThis_fake;
    }
    
    void teardown()
    {
       ClassName_Destroy(aClassName);
    }
};

TEST(ClassName, Fake)
{
    aClassName->virtualFunction_renameThis(aClassName);
    LONGS_EQUAL(1, fakeRan);
}

TEST(ClassName, Create)
{
  FAIL("Start here");
}

