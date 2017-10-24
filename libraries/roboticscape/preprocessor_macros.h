/*******************************************************************************
* preprocessor_macros.h
*
* This is a list of macros to clean up GNU-C extensions in a safe and portable
* way. Thank you Robert Love for providing this in "Linux System Programming",
* it is an excellent book.
*******************************************************************************/


#if __GNUC__ >= 3
# undef inline
# define inline			inline __attribute__ ((always_inline))
# define __noinline		__attribute__ ((noinline))
# define __pure			__attribute__ ((pure))
# define __const		__attribute__ ((const))
# define __noreturn		__attribute__ ((noreturn))
# define __malloc		__attribute__ ((malloc))
# define __must_check	__attribute__ ((warn_unused_result))
# define __deprecated	__attribute__ ((deprecated))
# define __used			__attribute__ ((used))
# define __unused		__attribute__ ((unused))
# define __packed		__attribute__ ((packed))
# define __align(x)		__attribute__ ((aligned (x)))
# define __align_max	__attribute__ ((aligned))
# define likely(x)		__builtin_expect (!!(x), 1)
# define unlikely(x)	__builtin_expect (!!(x), 0)
#else
# define __noinline		/* no noinline */
# define __pure			/* pure */
# define __const		/* const */
# define __noreturn		/* noreturn */
# define __malloc		/* malloc */
# define __must_check	/* warn_unused_result */
# define __deprecated	/* deprecated */
# define __used			/* used */
# define __unused		/* unused */
# define __packed		/* packed */
# define __align(x)		/* aligned */
# define __align_max	/* align_max */
# define likely(x) (x)
# define unlikely(x) (x)
#endif