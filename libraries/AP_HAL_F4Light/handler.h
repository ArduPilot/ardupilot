#pragma once

#include <hal.h>

template<typename T> void ZeroIt(T& value)
{
    memset(&value,0,sizeof(value));
}

FUNCTOR_TYPEDEF(MemberProcArg, void, uint32_t);

#pragma pack(push, 1)
union Revo_handler { // blood, bowels, assembler :) transform functors into a unified view for calling from C
    voidFuncPtr vp;
    revo_isr_handler isr;
    AP_HAL::Proc hp;
    AP_HAL::Device::PeriodicCb pcb;//\\  this is C not C ++, so we can not declare the support of functors explicitly, and are forced to pass
    AP_HAL::MemberProc mp;           //   support of functors explicitly, and are forced to pass
    MemberProcArg mpa;
    Handler h;          // treat as handle             <-- as 64-bit integer
    uint32_t w[2]; // words, to check.      if this is a functor then the high is the address of the flash and the lower one is the address in RAM.
    //                                      if this is a function pointer then lower word is an address in flash and high is 0
};
#pragma pack(pop)

