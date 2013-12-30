/*
  this can be used to check array indexes in vectors and matrices
 */

#ifndef MATH_ASSERT_H
#define MATH_ASSERT_H

#if MATH_CHECK_INDEXES
#include <assert.h>
#define ASSERT(x) assert(x)
#else
#define ASSERT(x) do {} while(0)
#endif

#endif // MATH_ASSERT_H

