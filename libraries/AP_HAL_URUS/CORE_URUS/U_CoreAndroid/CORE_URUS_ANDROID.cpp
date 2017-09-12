
#if defined(__ANDROID__)

#include "../CORE_URUS.h"
#include "../CORE_URUS_NAMESPACE.h"
#include "CORE_URUS_ANDROID.h"

#include "CoreUrusTimers_Android.h"
#include <stdio.h>

static CLCoreUrusTimers_Android coreTimers;

CORE_ANDROID::CORE_ANDROID() :
    NSCORE_URUS::CLCORE_URUS(&coreTimers)
{}

void CORE_ANDROID::init_core() const
{
    printf("android!\n");
}

const NSCORE_URUS::CLCORE_URUS& NSCORE_URUS::get_CORE()
{
    static const CORE_ANDROID _urus_core;
    return _urus_core;
}

#endif // __ANDROID__
