#pragma once

#if defined(HAVE_STDBOOL_H) && HAVE_STDBOOL_H
#include_next <stdbool.h>
#else

#define	__bool_true_false_are_defined	1

#ifndef __cplusplus

#define	bool	_Bool
#if __STDC_VERSION__ < 199901L && __GNUC__ < 3
typedef	int	_Bool;
#endif

#define	false	0
#define	true	1

#endif /* !__cplusplus */

#endif /* HAVE_STDBOOL_H */
