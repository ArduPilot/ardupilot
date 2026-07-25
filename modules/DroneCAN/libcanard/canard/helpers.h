#pragma once

#ifdef USE_USER_HELPERS
#include "canard_helpers_user.h"
#endif

#ifndef NOINLINE_FUNC
#define NOINLINE_FUNC __attribute__((noinline))
#endif

// define malloc and free
#ifndef CANARD_MALLOC
#include <stdlib.h>
#include <new>
#include <cstring>
#define CANARD_MALLOC malloc
#define CANARD_FREE free
#endif

namespace Canard {
template<typename T, typename ...Args>
T* allocate(Args...args) {
    auto ret = CANARD_MALLOC(sizeof(T));
    if (ret == nullptr) {
        return nullptr;
    }
    memset(ret, 0, sizeof(T));
    return new(ret) T(args...);
}

template<typename T>
void deallocate(T* ptr) {
    if (ptr == nullptr) {
        return;
    }
    ptr->~T();
    CANARD_FREE(ptr);
}

}