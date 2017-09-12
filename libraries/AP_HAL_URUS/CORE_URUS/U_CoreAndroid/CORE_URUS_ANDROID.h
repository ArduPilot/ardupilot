#pragma once

#if defined(__ANDROID__)

#include "../CORE_URUS.h"
#include "../CORE_URUS_NAMESPACE.h"

class CORE_ANDROID : public NSCORE_URUS::CLCORE_URUS {
public:
    CORE_ANDROID();
    void init_core() const override;
};

#endif // __ANDROID____
