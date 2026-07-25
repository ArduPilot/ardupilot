CppUTest on Symbian

Compliling

To compile CppUTest you need to have Symbian Posix libraries installed. On S60 it's 
recommended to use S60 Open C. On other platforms one can use Symbian PIPS. Compiling is
in standard way, by issuing from command line commands bldmake bldfiles && abld build in 
build-directory. You can also import the project's bld.inf file into Carbide in normal way.


Writing tests

CppUTest exports the needed header files into \epoc32\include\CppUTest -directory. Add the directory
to the include-path of the test project. One needs to include TestHarness.h file into test source
file. The test project must link against cpputest.lib using STATICLIBRARY statement in MMP-file. 
CppUTest depends also on standard C-library so the tests must be linked against libc as well.

The entry point file of the tests is normally the following:
#include <CppUTest/CommandLineTestRunner.h>

#include <stdio.h>

// This is a GCCE toolchain workaround needed when compiling with GCCE
// and using main() entry point
#ifdef __GCCE__
#include <staticlibinit_gcce.h>
#endif

int main(int argc, char** argv)
	{
	CommandLineTestRunner::RunAllTests(argc, argv);
	}

The test must be statically linked against libcrt0.lib if standard main (not E32Main) is used as above.

For the further example, please consult alltests.mmp in build directory.
