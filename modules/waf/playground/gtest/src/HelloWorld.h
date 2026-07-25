#ifndef HelloWorld_h_seen
#define HelloWorld_h_seen
#include <string>
class HelloWorld
{
  public:
    HelloWorld();
    HelloWorld(const std::string & msg);

    std::string message() const;
    void setMessage(const std::string & msg);
  private:
    std::string m_message;
};
#endif
