
#ifndef __FENV_POLYFILL_H__
#define __FENV_POLYFILL_H__

#if (defined __APPLE__) && (defined(__i386__) || defined(__x86_64__))

#include <fenv.h>

// Public domain polyfill for feenableexcept on OS X
// http://www-personal.umich.edu/~williams/archive/computation/fe-handling-example.c

inline int
feenableexcept (unsigned int excepts)
{
  static fenv_t fenv;
  unsigned int new_excepts = excepts & FE_ALL_EXCEPT,
               old_excepts;  // previous masks

  if ( fegetenv (&fenv) ) return -1;
  old_excepts = fenv.__control & FE_ALL_EXCEPT;

  // unmask
  fenv.__control &= ~new_excepts;
  fenv.__mxcsr   &= ~(new_excepts << 7);

  return ( fesetenv (&fenv) ? -1 : old_excepts );
}

#endif // APPLE

#endif // __FENV_POLYFILL_H__

