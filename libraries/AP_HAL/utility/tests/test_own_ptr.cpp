/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_gtest.h>

#include <utility>
#include <AP_HAL/utility/OwnPtr.h>

TEST(OwnPtrTest, SamePointer)
{
    int *a = new int{42};
    AP_HAL::OwnPtr<int> own(a);

    EXPECT_TRUE(own == a);
    EXPECT_TRUE(a == own);

    EXPECT_FALSE(own != a);
    EXPECT_FALSE(a != own);

    EXPECT_EQ(a, own.get());
}

TEST(OwnPtrTest, MoveOwnership)
{
    int *a = new int{42};
    AP_HAL::OwnPtr<int> own(a);
    AP_HAL::OwnPtr<int> own2 = std::move(own);

    EXPECT_EQ(own2.get(), a);
    EXPECT_EQ(own.get(), nullptr);
}

class TestDeleted {
public:
    TestDeleted(unsigned int &d) : deleted(d) { }
    ~TestDeleted() { deleted++; }

    unsigned int &deleted;
};

TEST(OwnPtrTest, DeleteOutOfScope)
{
    unsigned int deleted = 0;

    {
        AP_HAL::OwnPtr<TestDeleted> own(new TestDeleted{deleted});
    }

    EXPECT_EQ(deleted, 1U);
}

TEST(OwnPtrTest, DeleteOutOfScopeAfterMove)
{
    unsigned int deleted = 0;
    AP_HAL::OwnPtr<TestDeleted> own;

    {
        AP_HAL::OwnPtr<TestDeleted> own2(new TestDeleted{deleted});
        own = std::move(own2);
    }

    // own2 is now out of scope, but it has been moved already
    EXPECT_EQ(deleted, 0U);

    // now remove also 'own'
    own.clear();

    EXPECT_EQ(deleted, 1U);
}

class TestCall {
public:
    int foo() { return 42; }
};

TEST(OwnPtrTest, CallMethod)
{
    AP_HAL::OwnPtr<TestCall> own(new TestCall{});
    EXPECT_EQ(own->foo(), 42);
    EXPECT_EQ((*own).foo(), 42);
}

class TestDestructor {
public:
    TestDestructor(AP_HAL::OwnPtr<TestDeleted> v) : _v(std::move(v)) { }

    AP_HAL::OwnPtr<TestDeleted> _v;
};

TEST(OwnPtrTest, MoveToConstructor)
{
    unsigned int deleted = 0;

    AP_HAL::OwnPtr<TestDeleted> own(new TestDeleted{deleted});
    {
        EXPECT_EQ(0U, deleted);
        TestDestructor destructor{std::move(own)};
        EXPECT_EQ(0U, deleted);
    }
    EXPECT_EQ(1U, deleted);
}

static AP_HAL::OwnPtr<TestDeleted> create_test_deleted(unsigned int &deleted)
{
    return AP_HAL::OwnPtr<TestDeleted>(new TestDeleted(deleted));
}

TEST(OwnPtrTest, ReturnType)
{
    unsigned int deleted = 0;

    auto own = create_test_deleted(deleted);
    EXPECT_EQ(0U, deleted);
    {
        auto own2 = create_test_deleted(deleted);
        EXPECT_EQ(0U, deleted);
    }
    EXPECT_EQ(1U, deleted);
    own.clear();
    EXPECT_EQ(2U, deleted);
}

TEST(OwnPtrTest, ReplacePointer)
{
    unsigned int deleted1 = 0;
    unsigned int deleted2 = 0;

    auto own = create_test_deleted(deleted1);
    EXPECT_EQ(0U, deleted1);
    {
        own = create_test_deleted(deleted2);
        EXPECT_EQ(1U, deleted1);
    }
    EXPECT_EQ(0U, deleted2);
    own = nullptr;
    EXPECT_EQ(1U, deleted2);
}

TEST(OwnPtrTest, ReplaceWithRawPointer)
{
    unsigned int deleted1 = 0;

    auto own = create_test_deleted(deleted1);
    EXPECT_EQ(0U, deleted1);
    {
        own = new TestDeleted{deleted1};
        EXPECT_EQ(1U, deleted1);
    }
}

TEST(OwnPtrTest, Empty)
{
    int *a = new int{42};

    AP_HAL::OwnPtr<int> own1;
    EXPECT_FALSE(own1);

    own1 = a;
    EXPECT_TRUE((bool) own1);
}

class A {
public:
    A(int a) : _a(a) { }
    int _a;
};

class B : public A {
public:
    B() : A(42) { }
};

TEST(OwnPtrTest, Inheritance)
{
    A *a = new A(21);
    B *b = new B();
    AP_HAL::OwnPtr<A> own_a(a);
    AP_HAL::OwnPtr<B> own_b(b);

    own_a = std::move(own_b);
    EXPECT_EQ(b, own_a.get());
    EXPECT_EQ(42, own_a->_a);
}

AP_GTEST_MAIN()
