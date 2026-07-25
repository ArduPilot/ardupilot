#include "gtest/gtest.h"
#include "HelloWorld.h"
#include <string>

using namespace std;

TEST(HelloWorldTest, test0)
{
  HelloWorld hello;
  string expected("Hello World");
  ASSERT_EQ(expected, hello.message());
}

TEST(HelloWorldTest, test1)
{
  string expected("Hola Mundo");
  HelloWorld hello(expected);
  ASSERT_EQ(expected, hello.message());
  expected = "Hello, world!";
  hello.setMessage(expected);
  ASSERT_EQ(expected, hello.message());

}
