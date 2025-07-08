#include <AP_gtest.h>
#include <AP_Common/AP_Common.h>

int hal = 0;

class DummyDummy {
public:
    double d = 42.0;
    uint8_t count = 1;
};

TEST(AP_Common, TEST_CPP)
{
    DummyDummy * test_new = NEW_NOTHROW DummyDummy[1];
    EXPECT_FALSE(test_new == nullptr);
    EXPECT_TRUE(sizeof(test_new) == 8);
    EXPECT_FLOAT_EQ(test_new->count, 1);
    EXPECT_FLOAT_EQ(test_new->d, 42.0);

    DummyDummy * test_d = (DummyDummy*) ::operator new (sizeof(DummyDummy));
    EXPECT_FALSE(test_d == nullptr);
    EXPECT_TRUE(sizeof(test_d) == 8);
    EXPECT_EQ(test_d->count, 0);  // constructor isn't called
    EXPECT_FLOAT_EQ(test_d->d, 0.0);

    DummyDummy * test_d2 = NEW_NOTHROW DummyDummy;
    EXPECT_TRUE(sizeof(test_d2) == 8);
    EXPECT_EQ(test_d2->count, 1);
    EXPECT_FLOAT_EQ(test_d2->d, 42.0);

    delete[] test_new;
    delete test_d;
    delete test_d2;
}

AP_GTEST_MAIN()
