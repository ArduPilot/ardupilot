/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/util/lazy_constructor.hpp>


TEST(LazyConstructor, Basic)
{
    using ::uavcan::LazyConstructor;

    LazyConstructor<std::string> a;
    LazyConstructor<std::string> b;

    ASSERT_FALSE(a);
    ASSERT_FALSE(b.isConstructed());

    /*
     * Construction
     */
    a.destroy(); // no-op
    a.construct();
    b.construct<const char*>("Hello world");

    ASSERT_TRUE(a);
    ASSERT_TRUE(b.isConstructed());

    ASSERT_NE(*a, *b);
    ASSERT_STRNE(a->c_str(), b->c_str());

    ASSERT_EQ(*a, "");
    ASSERT_EQ(*b, "Hello world");

    /*
     * Copying
     */
    a = b;            // Assignment operator performs destruction and immediate copy construction
    ASSERT_EQ(*a, *b);
    ASSERT_EQ(*a, "Hello world");

    LazyConstructor<std::string> c(a); // Copy constructor call is forwarded to std::string

    ASSERT_EQ(*c, *a);

    *a = "123";
    ASSERT_NE(*c, *a);
    ASSERT_EQ(*c, *b);

    *c = "456";
    ASSERT_NE(*a, *c);
    ASSERT_NE(*b, *a);
    ASSERT_NE(*c, *b);

    /*
     * Destruction
     */
    ASSERT_TRUE(c);
    c.destroy();
    ASSERT_FALSE(c);
}
