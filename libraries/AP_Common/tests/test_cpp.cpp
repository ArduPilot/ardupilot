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

// These members intentionally rely on the zero-filling allocator, including
// across a user-provided constructor which only initialises one member.
struct AP_ZeroInitTest {
    AP_ZeroInitTest() : constructed(42) {}

    int untouched;
    int constructed;
};

TEST(AP_Common, NewTrivialZeroInitialisation)
{
    struct Plain {
        int untouched;
    };
    auto *p = NEW_NOTHROW Plain;
    ASSERT_NE(p, nullptr);
    // Copy the value so gtest's reference arguments do not keep the allocation alive.
    const int untouched = p->untouched;
    EXPECT_EQ(untouched, 0);
    delete p;
}

TEST(AP_Common, NewZeroInitialisation)
{
    auto *p = NEW_NOTHROW AP_ZeroInitTest;
    ASSERT_NE(p, nullptr);
    const int untouched = p->untouched;
    EXPECT_EQ(untouched, 0);
    EXPECT_EQ(p->constructed, 42);
    delete p;
}

TEST(AP_Common, NewArrayZeroInitialisation)
{
    auto *p = NEW_NOTHROW AP_ZeroInitTest[3];
    ASSERT_NE(p, nullptr);
    for (unsigned i = 0; i < 3; i++) {
        const int untouched = p[i].untouched;
        EXPECT_EQ(untouched, 0);
        EXPECT_EQ(p[i].constructed, 42);
    }
    delete[] p;
}

TEST(AP_Common, PlacementNewZeroInitialisation)
{
    alignas(AP_ZeroInitTest) unsigned char storage[sizeof(AP_ZeroInitTest)];
    memset(storage, 0, sizeof(storage));
    auto *p = new (storage) AP_ZeroInitTest;
    const int untouched = p->untouched;
    EXPECT_EQ(untouched, 0);
    EXPECT_EQ(p->constructed, 42);
    p->~AP_ZeroInitTest();
}

AP_GTEST_MAIN()
