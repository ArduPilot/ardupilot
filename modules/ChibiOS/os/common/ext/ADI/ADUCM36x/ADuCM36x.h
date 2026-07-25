#ifndef __ADUCM36x_H__
#define __ADUCM36x_H__

#if defined(ADUCM360)
#include "ADuCM360.h"
#elif defined(ADUCM361)
#include "ADuCM361.h"
#elif defined(ADUCM362)
#include "ADuCM362.h"
#elif defined(ADUCM363)
#include "ADuCM363.h"
#else
#include "ADuCM360.h"
#endif

#endif /* __ADUCM36x_H__ */
