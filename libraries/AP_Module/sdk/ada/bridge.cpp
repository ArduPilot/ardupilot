// SPDX-License-Identifier: GPL-3.0-or-later
#include <ap_module.hpp>

using ap_module::ContextBase;

extern "C" int32_t ap_module_ada_run(const ContextBase *context);
extern "C" int32_t ap_module_run(const ContextBase *context)
{
    return ap_module_ada_run(context);
}

extern "C" int32_t ap_module_ada_text(const ContextBase *context, int32_t severity, const char *message, uint32_t length)
{
    if (severity < 0 || severity > 7) {
        return int32_t(ap_module::Error::Invalid);
    }
    return int32_t(context->text(uint8_t(severity), ap_module::StringView(message, length)).error);
}

extern "C" int32_t ap_module_ada_stop_requested(const ContextBase *context)
{
    return context->stop_requested();
}

extern "C" uint64_t ap_module_ada_time_us(const ContextBase *context)
{
    return context->time_us();
}

extern "C" int32_t ap_module_ada_sleep_ms(const ContextBase *context, uint32_t milliseconds)
{
    return int32_t(context->sleep_ms(milliseconds).error);
}

extern "C" [[noreturn]] void ap_module_ada_assertion_failed()
{
    ap_module::fatal("Ada assertion failed");
}

// Preserve GNAT's runtime checks, terminating just this module on failure.
// Never provide a returning stub or link an exception-unwinding runtime.
#define ADA_CHECK(name) \
    extern "C" [[noreturn]] void __gnat_rcheck_ ## name(const char *, int) \
    { ap_module::fatal("Ada " #name); }

ADA_CHECK(CE_Access_Check)
ADA_CHECK(CE_Null_Access_Parameter)
ADA_CHECK(CE_Discriminant_Check)
ADA_CHECK(CE_Divide_By_Zero)
ADA_CHECK(CE_Explicit_Raise)
ADA_CHECK(CE_Index_Check)
ADA_CHECK(CE_Invalid_Data)
ADA_CHECK(CE_Length_Check)
ADA_CHECK(CE_Null_Exception_Id)
ADA_CHECK(CE_Null_Not_Allowed)
ADA_CHECK(CE_Overflow_Check)
ADA_CHECK(CE_Partition_Check)
ADA_CHECK(CE_Range_Check)
ADA_CHECK(CE_Tag_Check)
ADA_CHECK(PE_Access_Before_Elaboration)
ADA_CHECK(PE_Accessibility_Check)
ADA_CHECK(PE_Explicit_Raise)
ADA_CHECK(PE_Implicit_Return)
ADA_CHECK(PE_Misaligned_Address_Value)
ADA_CHECK(PE_Missing_Return)
ADA_CHECK(PE_Raise_Check)
ADA_CHECK(SE_Empty_Storage_Pool)
ADA_CHECK(SE_Explicit_Raise)
ADA_CHECK(SE_Infinite_Recursion)
ADA_CHECK(SE_Object_Too_Large)
#undef ADA_CHECK

extern "C" [[noreturn]] void __gnat_last_chance_handler(const char *, int)
{
    ap_module::fatal("Ada last chance handler");
}
