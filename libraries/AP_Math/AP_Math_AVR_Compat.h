// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_MATH_AVR_COMPAT_H
#define AP_MATH_AVR_COMPAT_H

// This file defines the floating-point version of standard C math
// functions on doubles, if they are not present in avr-libc.

#ifndef cosf
# define cosf cos
#endif

#ifndef sinf
# define sinf sin
#endif

#ifndef tanf
# define tanf tan
#endif

#ifndef fabsf
# define fabsf fabs
#endif

#ifndef fmodf
# define fmodf fmod
#endif

#ifndef sqrtf
# define sqrtf sqrt
#endif

#ifndef cbrtf
# define cbrtf cbrt
#endif

#ifndef hypotf
# define hypotf hypot
#endif

#ifndef squaref
# define squaref square
#endif

#ifndef floorf
# define floorf floor
#endif

#ifndef ceilf
# define ceilf ceil
#endif

#ifndef frexpf
# define frexpf frexp
#endif

#ifndef ldexpf
# define ldexpf ldexp
#endif

#ifndef expf
# define expf exp
#endif

#ifndef coshf
# define coshf cosh
#endif

#ifndef sinhf
# define sinhf sinh
#endif

#ifndef tanhf
# define tanhf tanh
#endif

#ifndef acosf
# define acosf acos
#endif

#ifndef asinf
# define asinf asin
#endif

#ifndef atanf
# define atanf atan
#endif

#ifndef atan2f
# define atan2f atan2
#endif

#ifndef logf
# define logf log
#endif

#ifndef log10f
# define log10f log10
#endif

#ifndef powf
# define powf pow
#endif

#ifndef isnanf
# define isnanf isnan
#endif

#ifndef isinff
# define isinff isinf
#endif

#ifndef isfinitef
# define isfinitef isfinite
#endif

#ifndef copysignf
# define copysignf copysign
#endif

#ifndef signbitf
# define signbitf signbit
#endif

#ifndef fdimf
# define fdimf fdim
#endif

#ifndef fmaf
# define fmaf fma
#endif

#ifndef fminf
# define fminf fmin
#endif

#ifndef truncf
# define truncf trunc
#endif

#ifndef roundf
# define roundf round
#endif

#ifndef lroundf
# define lroundf lround
#endif

#ifndef lrintf
# define lrintf lrint
#endif

#endif  // !defined AP_MATH_AVR_COMPAT_H
