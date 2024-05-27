#include <AP_gtest.h>

/*
  tests for AP_NavEKF/EKF_Buffer.cpp
 */

#include <AP_NavEKF/EKF_Buffer.h>
#include <stdlib.h>

#include <AP_HAL/AP_HAL.h>
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX

TEST(EKF_Buffer, EKF_Buffer)
{
    struct test_data : EKF_obs_element_t {
        uint32_t data;
    };
    EKF_obs_buffer_t<test_data> buf;
    buf.init(8);
    struct test_data d, d2;
    uint32_t now = 100;
    d.data = 17;
    d.time_ms = now++;
    buf.push(d);

    EXPECT_TRUE(buf.recall(d2, now));
    EXPECT_EQ(d2.data, d.data);
    EXPECT_EQ(d2.time_ms, uint32_t(100));

    d.time_ms = 101;
    buf.push(d);
    d.time_ms = 102;
    buf.push(d);

    // recall first element
    EXPECT_TRUE(buf.recall(d2, 101));
    EXPECT_EQ(d2.data, d.data);
    EXPECT_EQ(d2.time_ms, uint32_t(101));

    EXPECT_TRUE(buf.recall(d2, 102));
    EXPECT_EQ(d2.data, d.data);
    EXPECT_EQ(d2.time_ms, uint32_t(102));

    EXPECT_FALSE(buf.recall(d2, 103));

    d.time_ms = 101;
    buf.push(d);
    d.time_ms = 102;
    buf.push(d);

    // recall 2nd element, note that first element is discarded
    EXPECT_TRUE(buf.recall(d2, 103));
    EXPECT_EQ(d2.data, d.data);
    EXPECT_EQ(d2.time_ms, uint32_t(102));

    EXPECT_FALSE(buf.recall(d2, 103));
    EXPECT_FALSE(buf.recall(d2, 103));

    // test overflow of buffer
    for (uint8_t i=0; i<16; i++) {
        d.time_ms = 100+i;
        d.data = 1000+i;
        buf.push(d);
    }
    for (uint8_t i=0; i<16; i++) {
        if (i < 8) {
            EXPECT_TRUE(buf.recall(d2, 108+i));
        } else {
            EXPECT_FALSE(buf.recall(d2, 1100));
        }
    }

    // test with an element that is too old
    d.time_ms = 101;
    d.data = 200;
    buf.push(d);
    d.time_ms = 1002;
    d.data = 2000;
    buf.push(d);

    // recall element, first one is too old so will be discarded
    EXPECT_TRUE(buf.recall(d2, 1003));
    EXPECT_EQ(d2.data, 2000U);
    EXPECT_EQ(d2.time_ms, uint32_t(1002));

    EXPECT_FALSE(buf.recall(d2, 103));


    // test 32 bit time wrap
    d.time_ms = 0xFFFFFFF0U;
    d.data = 200;
    buf.push(d);
    d.time_ms = 0xFFFFFFFAU;
    d.data = 2000;
    buf.push(d);
    d.time_ms = 0xFFFFFFFBU;
    d.data = 2001;
    buf.push(d);

    // recall element, first one is too old so will be discarded
    EXPECT_TRUE(buf.recall(d2, 10));
    EXPECT_EQ(d2.data, 2001U);
    EXPECT_EQ(d2.time_ms, uint32_t(0xFFFFFFFBU));

    EXPECT_FALSE(buf.recall(d2, 103));

    // test 32 bit time wrap with a too old element
    d.time_ms = 0xFFFFFFFFU - 1000U;
    d.data = 200;
    buf.push(d);
    d.time_ms = 0xFFFFFFFBU;
    d.data = 2001;
    buf.push(d);

    // recall element, first one is too old so will be discarded
    EXPECT_TRUE(buf.recall(d2, 10));
    EXPECT_EQ(d2.data, 2001U);
    EXPECT_EQ(d2.time_ms, uint32_t(0xFFFFFFFBU));

    EXPECT_FALSE(buf.recall(d2, 103));
}

TEST(ekf_imu_buffer, one_element_case)
{
    // test degenerate 1-element case:
    struct element {
        uint8_t value;
    };
    ekf_imu_buffer *b = NEW_NOTHROW ekf_imu_buffer(sizeof(element));
    b->init(1);  // 1 element
    EXPECT_EQ(b->is_filled(), false);
    EXPECT_EQ(b->get_oldest_index(), b->get_youngest_index());
    const element e { 34 };
    b->push_youngest_element((void*)&e);
    EXPECT_EQ(b->is_filled(), true);
    EXPECT_EQ(b->get_oldest_index(), b->get_youngest_index());
    element returned_element {};
    b->get_oldest_element((void*)&returned_element);
    EXPECT_EQ(e.value, returned_element.value);
    element *another_returned_element = (element*)b->get(0);
    EXPECT_EQ(e.value, another_returned_element->value);
    // we don't do bounds checking, so get here returns non-nullptr:
    // another_returned_element = (element*)b.get(17);
    // EXPECT_EQ(another_returned_element, nullptr);
    b->reset();
    EXPECT_EQ(b->get_oldest_index(), b->get_youngest_index());
    // we don't do bounds checking, so get here returns non-nullptr:
    // another_returned_element = (element*)b.get(0);
    // EXPECT_EQ(another_returned_element, nullptr);
}

TEST(ekf_imu_buffer, is_filled)
{
    // https://github.com/ArduPilot/ardupilot/issues/25316
    struct element {
        uint8_t value;
    };
    ekf_imu_buffer *b = NEW_NOTHROW ekf_imu_buffer(sizeof(element));
    b->init(4);  // 4 elements
    const element e { 34 };

    EXPECT_EQ(b->is_filled(), false);

    b->push_youngest_element((void*)&e);
    EXPECT_EQ(b->is_filled(), false);

    b->push_youngest_element((void*)&e);
    EXPECT_EQ(b->is_filled(), false);

    b->push_youngest_element((void*)&e);
    EXPECT_EQ(b->is_filled(), false);

    b->push_youngest_element((void*)&e);
    EXPECT_EQ(b->is_filled(), true);
}

AP_GTEST_MAIN()

#endif // HAL_SITL or HAL_LINUX
