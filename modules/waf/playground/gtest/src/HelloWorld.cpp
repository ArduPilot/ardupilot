
#include "HelloWorld.h"
HelloWorld::HelloWorld():m_message("Hello World")
{
}

HelloWorld::HelloWorld(const std::string & msg):m_message(msg)
{}

std::string HelloWorld::message() const
{
  return m_message;
}

void HelloWorld::setMessage(const std::string & msg)
{
  m_message = msg;
}
