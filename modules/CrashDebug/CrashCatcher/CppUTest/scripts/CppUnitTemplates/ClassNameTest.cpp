#include <cppunit/config/SourcePrefix.h>
#include <cppunit/extensions/HelperMacros.h>
#include "ClassName.h"

class ClassNameTest: public CPPUNIT_NS::TestFixture
{
    CPPUNIT_TEST_SUITE(ClassNameTest);
    CPPUNIT_TEST(testCreate);
    CPPUNIT_TEST_SUITE_END();

  ClassName* aClassName;

public:

  void setUp()
  {
    aClassName = new ClassName();
  }

  void tearDown()
  {
    delete aClassName;
  }

  void testCreate()
  {
    CPPUNIT_FAIL("Start here");
  }
};

CPPUNIT_TEST_SUITE_REGISTRATION(ClassNameTest);


