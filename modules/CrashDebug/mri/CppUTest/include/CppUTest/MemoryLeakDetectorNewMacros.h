
/*
 * This file can be used to get extra debugging information about memory leaks in your production code.
 * It defines a preprocessor macro for operator new. This will pass additional information to the
 * operator new and this will give the line/file information of the memory leaks in your code.
 *
 * You can use this by including this file to all your production code. When using gcc, you can use
 * the -include file to do this for you.
 *
 * Warning: Using the new macro can cause a conflict with newly declared operator news. This can be
 * resolved by:
 * 1. #undef operator new before including this file
 * 2. Including the files that override operator new before this file.
 *    This can be done by creating your own NewMacros.h file that includes your operator new overrides
 *    and THEN this file.
 *
 * STL (or StdC++ lib) also does overrides for operator new. Therefore, you'd need to include the STL
 * files *before* this file too.
 *
 */

/* Warning for maintainers:
 * This macro code is duplicate from TestHarness.h. The reason for this is to make the two files
 * completely independent from each other. NewMacros file can be included in production code whereas
 * TestHarness.h is only included in test code.
 */

#include "StandardCLibrary.h"

#ifndef CPPUTEST_USE_MEM_LEAK_DETECTION
#ifdef CPPUTEST_MEM_LEAK_DETECTION_DISABLED
#define CPPUTEST_USE_MEM_LEAK_DETECTION 0
#else
#define CPPUTEST_USE_MEM_LEAK_DETECTION 1
#endif
#endif

#if CPPUTEST_USE_MEM_LEAK_DETECTION

#ifndef CPPUTEST_USE_STD_CPP_LIB
#ifdef CPPUTEST_STD_CPP_LIB_DISABLED
#define CPPUTEST_USE_STD_CPP_LIB 0
#else
#define CPPUTEST_USE_STD_CPP_LIB 1
#endif
#endif

/* This #ifndef prevents <new> from being included twice and enables the file to be included anywhere */
#ifndef CPPUTEST_USE_NEW_MACROS

	#if CPPUTEST_USE_STD_CPP_LIB
	#include <new>
	#include <memory>
	#include <string>

		void* operator new(size_t size, const char* file, int line) throw (std::bad_alloc);
		void* operator new[](size_t size, const char* file, int line) throw (std::bad_alloc);
		void* operator new(size_t size) throw(std::bad_alloc);
		void* operator new[](size_t size) throw(std::bad_alloc);

	#else

		void* operator new(size_t size, const char* file, int line);
		void* operator new[](size_t size, const char* file, int line);
		void* operator new(size_t size);
		void* operator new[](size_t size);
	#endif
#endif

#define new new(__FILE__, __LINE__)

#define CPPUTEST_USE_NEW_MACROS 1

#endif
