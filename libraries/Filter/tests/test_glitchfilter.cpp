#include <AP_gtest.h>

#include <Filter/GlitchFilter.h>

#ifndef GTEST_COUT
    #define GTEST_COUT std::cerr << "[          ] [ INFO ]"
#endif



// TEST(GlitchFilterTest, Validate_Range)
// {
//     GlitchFilter test_filter;
//     test_filter.init(0.1);

//     // bad range = -1 because it is negative value. 
//     test_filter.reset();
//     EXPECT_EQ(true,test_filter.is_glitch(-1,10)); // bad range
    
//     // first value (10) is accepted as there is no other samples 
//     test_filter.reset();
//     EXPECT_EQ(false,test_filter.is_glitch(1,10)); // (not a glitch)
// }



TEST(GlitchFilterTest, Glitch_MINIMUM_CHANGES)
{
    // Simulating a function that changes +/-1.0 % of the value each step
    // Filter range accepts only 1.0 % so it accept all readings.

    GlitchFilter test_filter;
    test_filter.init(0.1, 1.0f, 10.0f);

    // any value will be accepted after reset()
    float value  = 100.0; // any positive value.
    test_filter.reset();
    EXPECT_EQ(false,test_filter.is_glitch(5,value));
    
    
    for (int i=0; i<20; ++i)
    {
        if ((i % 2) ==0)
        {
            value -= (value * 0.008f);
        }
        else
        {
            value += (value * 0.008f);
        }
        EXPECT_EQ(false,test_filter.is_glitch(1.0f , value));
    }
}


TEST(GlitchFilterTest, Glitch_LARGE_Peaks)
{
    // Simulating a function that changes +/- 1.0f % of the pressure each step
    // but every 20 step there is a peak +10.0f % that is ten times higher.
    // Filter range accepts only 1.0f% so it reject the peaks.

    GlitchFilter test_filter;
    test_filter.init(0.9, 1.0f, 10.0f);

    // any value will be accepted after reset()
    float value = 100.0; // any positive value.
    test_filter.reset();
    
    
    for (int i=0; i<1000; ++i)
    {
        bool expected_res = false;
        if ((i!=0) && ((i % 50) ==0))
        {   // simulate a peak every 20 sample skip sample 0
            expected_res = true;
            const float peak_value = value + (value * 0.50f);
            GTEST_COUT << "Peak value " << peak_value << std::endl;
            bool res = test_filter.is_glitch(3.0f , peak_value);
            if (res == expected_res)
            {
                GTEST_COUT << "Glitch & Ignored" <<  std::endl;
            }
            EXPECT_EQ(expected_res,res);
        }
        else 
        {
            if ((i % 2) ==0)
            {
                value -= (value * 0.008f);
            }
            else
            {
                value += (value * 0.008f);
            }
            EXPECT_EQ(expected_res,test_filter.is_glitch(3.0f , value));
        }
    }
}


TEST(GlitchFilterTest, Glitch_Raising)
{
    // Simulating a function that changes +/- 1.0f % of the pressure each step
    // but every 20 step there is a peak +10.0f % that is ten times higher.
    // Filter range accepts only 1.0f% so it reject the peaks.

    GlitchFilter test_filter;
    test_filter.init(0.9, 1.0f, 10.0f);

    // any value will be accepted after reset()
    float value = 100.0; // any positive value.
    test_filter.reset();
    
    
    for (int i=0; i<1000; ++i)
    {
        bool expected_res = false;
        if ((i % 2) ==0)
        {
            value -= (value * 0.001f);
        }
        else
        {
            value += (value * 0.01f);
        }
        
        EXPECT_EQ(expected_res,test_filter.is_glitch(3.0f , value));
    }
}




AP_GTEST_MAIN()
