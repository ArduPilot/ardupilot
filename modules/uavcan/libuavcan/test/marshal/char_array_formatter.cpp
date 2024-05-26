/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/marshal/char_array_formatter.hpp>

using uavcan::Array;
using uavcan::ArrayModeDynamic;
using uavcan::ArrayModeStatic;
using uavcan::IntegerSpec;
using uavcan::SignednessSigned;
using uavcan::SignednessUnsigned;
using uavcan::CastModeSaturate;
using uavcan::CastModeTruncate;
using uavcan::CharArrayFormatter;

TEST(CharArrayFormatter, Basic)
{
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 45> A8;
    A8 a;

    CharArrayFormatter<A8> f(a);
    ASSERT_STREQ("", f.getArray().c_str());

    f.write("Don't %s.", "Panic");
    ASSERT_STREQ("Don't Panic.", f.getArray().c_str());

    f.write(" abc%idef ", 123);
    ASSERT_STREQ("Don't Panic. abc123def ", f.getArray().c_str());

    f.write("%g", 0.0);
    ASSERT_STREQ("Don't Panic. abc123def 0", f.getArray().c_str());

    a.clear();
    ASSERT_STREQ("", f.getArray().c_str());

    f.write("123456789");
    ASSERT_STREQ("123456789", f.getArray().c_str());
}

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

TEST(CharArrayFormatter, Hardcore)
{
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 150> A8;
    A8 a;

    CharArrayFormatter<A8> f(a);

    f.write(
        "%% char='%*' double='%*' long='%*' unsigned long='%*' int='%s' long double='%*' bool='%*' const char='%*' %%",
        '%', -12.3456, -123456789123456789L, 987654321, -123456789, 0.000000001L, true, "Don't Panic.");

    static const std::string Reference =
        "% char='%' double='-12.3456' long='-123456789123456789' unsigned long='987654321' int='-123456789' "
        "long double='1e-09' bool='1' const char='Don't Pani"; // few chars truncated!

    ASSERT_STREQ(Reference.c_str(), f.getArray().c_str());

    a.clear();

    f.write("");
    ASSERT_STREQ("", f.getArray().c_str());

    f.write("%%");                                   // Nothing to format --> "%%" is not expanded
    ASSERT_STREQ("%%", f.getArray().c_str());

    f.write("%*", "Test", 123, true);                // Extra args ignored
    ASSERT_STREQ("%%Test", f.getArray().c_str());

    f.write("%% %* %* %% %*", true);                 // Insufficient args are OK; second "%%" is not expanded
    ASSERT_STREQ("%%Test% 1 %* %% %*", f.getArray().c_str());
}

#endif
