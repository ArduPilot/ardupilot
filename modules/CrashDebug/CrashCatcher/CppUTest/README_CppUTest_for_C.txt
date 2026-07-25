
CPPUTest can and has been used for testing C code also. When testing
C code there are a couple of things to keep in mind and a couple of
common problems to solve.


---++ Using extern "C"

When including C-header files or when declaring C-variables and routines
in a .cpp file, you'll have to surround them with an extern "C". This is
because the C++ linker works different than the C linker and you need to
instruct the compiler about this. If you do NOT do this, you will probably
get a linker error, like unresolved symbols, for a routine that you did
implement.

An example:

extern "C" {
	#include "hello.h"
	
	extern HelloWorldApi theRealHelloWorldApi;
}

---++ CppUTest support for C

CppUTest comes with a file called TestHarness_c.h which contains a couple
of routines that can be used in C code, like C-versions of the CHECK-MARCO's.
The file also contains malloc and free routines that can be used for using
the CppUTest memory leak detector. These routines should be used instead of
the normal malloc/free. This can be achieved by #defining them somewhere, for
examples as a compiler option: -Dmalloc=cpputest_malloc.

It's important to remember that TestHarness_c.h is a C-header file. It can be
used in C code, but when using in C++ code, you need to use extern "C" before
including it.


---++ C++ keywords used

It sometimes happens that a C file uses a C++ keyword as a type or something
else. The most common one is the bool-type. This can typically be solved by
#defining the bool to something else. For example:

extern "C" {
#define bool helloBool
#include "hello.h"
#undef bool
}

The #undef is optional. It is possible that this solution leads to problems in
some situation (never happened to me). The same solution works for other C++
key-words

---++ Other

   * In C, sometimes people use empty structs. The sizeof(empty struct) would be
     0. In C++, the sizeof(empty struct) would be something. The best way to fix
     this is to not use empty structs in C.
     According to http://www.glenmccl.com/bett.htm an empty stuct in C is illegal
     anyway.
     
---++ References

   * http://www.glenmccl.com/bett.htm
     Describes some differences between C and C++
   * http://en.wikipedia.org/wiki/Compatibility_of_C_and_C%2B%2B
     Wikipedia entry on the compatibility between C and C++
     