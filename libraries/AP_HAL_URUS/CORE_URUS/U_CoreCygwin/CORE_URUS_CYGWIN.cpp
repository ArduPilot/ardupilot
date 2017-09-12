
#if defined(__CYGWIN__)

#include "../CORE_URUS.h"
#include "../CORE_URUS_NAMESPACE.h"
#include "CORE_URUS_CYGWIN.h"

#include "CoreUrusTimers_Cygwin.h"
#include "CoreUrusScheduler_Cygwin.h"
#include "CoreUrusUARTDriver_Cygwin.h"
#include <stdio.h>

static CLCoreUrusTimers_Cygwin coreTimers;
static CLCoreUrusScheduler_Cygwin coreScheduler;
static CLCoreUrusUARTDriver_Cygwin coreUARTDriver(0, true);

CORE_CYGWIN::CORE_CYGWIN() :
    NSCORE_URUS::CLCORE_URUS(
        &coreTimers)
{}

void CORE_CYGWIN::init_core() const
{
#if 0
    printf("Cygwin Core Started!\n");
#endif
}

NSCORE_URUS::CLCoreUrusScheduler* NSCORE_URUS::get_scheduler()
{
    return &coreScheduler;
}

NSCORE_URUS::CLCoreUrusUARTDriver* NSCORE_URUS::get_uartDriver()
{
    return &coreUARTDriver;
}

const NSCORE_URUS::CLCORE_URUS& NSCORE_URUS::get_CORE()
{
    static const CORE_CYGWIN _urus_core;
    return _urus_core;
}

#endif // __CYGWIN__
