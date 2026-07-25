#include <cppunit/extensions/HelperMacros.h>
#include "HelloWorld.h"
#include <string>

using namespace std;

class HelloWorldTest : public CPPUNIT_NS::TestFixture
{
  private:
    CPPUNIT_TEST_SUITE( HelloWorldTest );
    CPPUNIT_TEST( test0 );
    CPPUNIT_TEST( test1 );
    CPPUNIT_TEST_SUITE_END();

  public:
    void test0();
    void test1();

};

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( HelloWorldTest );

void HelloWorldTest::test0()
{
  HelloWorld hello;
  string expected("Hello World");
  CPPUNIT_ASSERT_EQUAL(expected, hello.message());
}

void HelloWorldTest::test1()
{
  string expected("Hola Mundo");
  HelloWorld hello(expected);
  CPPUNIT_ASSERT_EQUAL(expected, hello.message());
  expected = "Hello, world!";
  hello.setMessage(expected);
  CPPUNIT_ASSERT_EQUAL(expected, hello.message());

}
