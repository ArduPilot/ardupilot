#include <AP_gtest.h>

#include <AP_Common/Bitmask.h>

TEST(Bitmask, Tests)
{
    Bitmask<49> x;

    EXPECT_EQ(-1, x.first_set());
    x.set(5);
    EXPECT_EQ(5, x.first_set());
    x.clear(5);
    EXPECT_EQ(-1, x.first_set());

    EXPECT_EQ(-1, x.first_set());
    x.set(42);
    EXPECT_EQ(42, x.first_set());
    x.clear(42);
    EXPECT_EQ(-1, x.first_set());

    EXPECT_EQ(-1, x.first_set());
    x.set(0);
    x.set(5);
    x.set(6);
    x.set(48);
    EXPECT_EQ(0, x.first_set());
    EXPECT_EQ(0, x.first_set());
    x.clear(0);
    EXPECT_EQ(5, x.first_set());
    EXPECT_EQ(5, x.first_set());
    x.clear(5);
    EXPECT_EQ(6, x.first_set());
    EXPECT_EQ(6, x.first_set());
    x.clear(6);
    EXPECT_EQ(48, x.first_set());
    EXPECT_EQ(48, x.first_set());
    x.clear(48);
    EXPECT_EQ(-1, x.first_set());
}

TEST(Bitmask, SetAll)
{
    Bitmask<49> x;
    EXPECT_EQ(-1, x.first_set());
    EXPECT_EQ(false, x.get(45));
    x.setall();
    EXPECT_EQ(0, x.first_set());
    x.clear(0);
    EXPECT_EQ(1, x.first_set());
    x.clear(1);
    EXPECT_EQ(2, x.first_set());
    EXPECT_EQ(true, x.get(45));
    EXPECT_EQ(false, x.empty());
    x.clearall();
    EXPECT_EQ(-1, x.first_set());
    EXPECT_EQ(false, x.get(45));
    EXPECT_EQ(true, x.empty());
}

TEST(Bitmask, Assignment)
{
    Bitmask<49> x;
    x.set(0);
    x.set(5);
    x.set(6);
    x.set(48);

    Bitmask<49> y;
    y = x;
    x.clear(0);
    EXPECT_EQ(true, y.get(0));
    EXPECT_EQ(true, y.get(5));
    EXPECT_EQ(true, y.get(6));
    EXPECT_EQ(true, y.get(48));
}

AP_GTEST_MAIN()

int hal = 0; // bizarrely, this fixes an undefined-symbol error but doesn't raise a type exception.  Yay.
