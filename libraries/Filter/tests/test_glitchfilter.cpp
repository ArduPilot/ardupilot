#include <AP_gtest.h>

#include <Filter/GlitchFilter.h>




TEST(GlitchFilterTest, Validate_Range)
{
    GlitchFilter test_filter;
    test_filter.init(0.1);

    // bad range
    test_filter.reset();
    EXPECT_EQ(false,test_filter.is_glitch(-1,10));
    
    // first value (10) is accepted as there is no other samples
    test_filter.reset();
    EXPECT_EQ(true,test_filter.is_glitch(1,10));
}



TEST(GlitchFilterTest, Glitch_SMALL)
{
    GlitchFilter test_filter;
    test_filter.init(0.1);

    // bad range
    test_filter.reset();
    EXPECT_EQ(true,test_filter.is_glitch(5,99545.0));
    
    for (int i=10; i<15; ++i)
    {
        EXPECT_EQ(true,test_filter.is_glitch(100,99545.0 + ( 1.0f / i)));
    }
}

TEST(GlitchFilterTest, Glitch_LARGE)
{
    GlitchFilter test_filter;
    test_filter.init(0.1);

    // bad range
    test_filter.reset();
    EXPECT_EQ(true,test_filter.is_glitch(5,99545.0));
    
    for (int i=10; i<15; ++i)
    {
        EXPECT_EQ(false,test_filter.is_glitch(100,99545.0 + 100 * i));
    }
}

AP_GTEST_MAIN()
