#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Common/BufferHandler.h>

#define EXPECT_NO_DEATH(EXPR) EXPECT_EXIT({EXPR; exit(0);}, ::testing::ExitedWithCode(0), "");

TEST(BufferHandler, CtorWithAlloc) {

    EXPECT_EQ((BufferHandler<int>    {nullptr, 42U}).size(), 42U);
    EXPECT_EQ((BufferHandler<int>    {42U}         ).size(), 42U);
    EXPECT_EQ((BufferHandler<int, 2> {8U}          ).size(), 8U*2);
}

TEST(BufferHandler, ViewCtor)
{
    int buf[3] {10,20,30};
    BufferHandler<int> view {buf, 3};
    EXPECT_EQ(buf, view.ptr());
}


TEST(BufferHandler, Move) {

    BufferHandler<int> owner {42};
    auto ptr = owner.ptr();
    {
        BufferHandler<int> view {std::move(owner)};
        EXPECT_EQ(nullptr, owner.ptr());
        EXPECT_EQ(ptr, view.ptr());
        EXPECT_NO_DEATH(std::ignore = *view.ptr());
    }
}

TEST(BufferHandler, Block) {

    int buf[3] {10,20,30};
    BufferHandler<int, 3> view {buf, 1};

    EXPECT_EQ(buf, view.ptr());
    EXPECT_EQ(*view.block<0>(), 10);
    EXPECT_EQ(*view.block<1>(), 20);
    EXPECT_EQ(*view.block<2>(), 30);
    EXPECT_EQ(view.block<3>(), nullptr);
}

AP_GTEST_MAIN()
