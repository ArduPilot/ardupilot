#include <iostream>
#include "MyLib/TestClass.h"
#include "config.h"

int main(int argc, char const *argv[])
{
	TestClass a;
	std::cout << a.message() << std::endl;
  std::cout << "Number should be 10: " << NUMBER << std::endl;


	return 0;
}