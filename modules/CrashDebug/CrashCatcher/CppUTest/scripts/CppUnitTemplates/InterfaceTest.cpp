#include <cppunit/config/SourcePrefix.h>
#include <cppunit/extensions/HelperMacros.h>
#include "ClassName.h"
#include "MockClassName.h"

class MockClassNameTest: public CPPUNIT_NS::TestFixture
{
    CPPUNIT_TEST_SUITE(MockClassNameTest);
    CPPUNIT_TEST(testCreate);
    CPPUNIT_TEST_SUITE_END();

    ClassName* aClassName;
    MockClassName* mockClassName;

public:

    void setUp()
    {
        mockClassName = new MockClassName();
        aClassName = mockClassName;
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

CPPUNIT_TEST_SUITE_REGISTRATION(MockClassNameTest);
