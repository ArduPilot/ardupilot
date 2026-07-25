#include "Python.h"
#include <iostream>

#include "lib.h"
// cython function
#include "cy_cxxtest_api.h"

int main(int argc, char** argv) 
{
  std::cout << "::: cxx-app\n";
  Py_Initialize();
  if (!Py_IsInitialized()) {
    std::cerr << ":: could not initialize python interpreter !\n";
    return 1;
  } else {
    std::cout << ":: python initialized\n";
  }
  PyEval_InitThreads();
  if (!PyEval_ThreadsInitialized()) {
    std::cerr << ":: could not init GIL !\n";
    return 1;
  }

  if (import_cy_cxxtest()) {
    std::cerr << "** could not import 'cy_cxxtest' module !\n";
    return 1;
  } else {
    std::cout << "::: successfully imported 'cy_cxxtest'\n";
  }

  cy_hello();
  std::cout << "::: cxx-app [done]\n";
 return 0;
}
