#include <assert.h>

#include "HAL.h"

namespace AP_HAL {

HAL::FunCallbacks::FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void))
    : _setup(setup_fun)
    , _loop(loop_fun)
{
}

}
