#pragma once

#if defined(__CYGWIN__)

#include "../CORE_URUS.h"
#include "../CORE_URUS_NAMESPACE.h"

class CORE_CYGWIN : public NSCORE_URUS::CLCORE_URUS {
public:
    CORE_CYGWIN();
    void init_core() const override;
};

#endif // __CYGWIN__
