
#if defined(__CYGWIN__)

#include "../CORE_URUS.h"
#include "../CORE_URUS_NAMESPACE.h"
#include "CORE_URUS_CYGWIN.h"

#include "CoreUrusTimers_Cygwin.h"
#include <stdio.h>

static CLCoreUrusTimers_Cygwin coreTimers;

CORE_CYGWIN::CORE_CYGWIN() :
    NSCORE_URUS::CLCORE_URUS(&coreTimers)
{}

void CORE_CYGWIN::init_core() const
{
    printf("cygwin!\n");
}

const NSCORE_URUS::CLCORE_URUS& NSCORE_URUS::get_CORE()
{
    static const CORE_CYGWIN _urus_core;
    return _urus_core;
}

#endif // __CYGWIN__
