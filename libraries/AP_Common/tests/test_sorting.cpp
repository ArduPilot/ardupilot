#include <AP_gtest.h>

/*
  tests for AP_Common/sorting.cpp
 */

#include <AP_Common/AP_Common.h>
#include <AP_Common/sorting.h>
#include <stdlib.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX

static int comp16(const uint16_t *v1, const uint16_t *v2) {
    return int32_t(*v1) - int32_t(*v2);
}

static void check_equal(const uint16_t *a1, const uint16_t *a2, uint16_t n)
{
    for (uint8_t j=0; j<n; j++) {
        EXPECT_EQ(a1[j], a2[j]);
    }
}

TEST(Sorting, sort)
{
    for (uint16_t i=0; i<10000; i++) {
        const uint8_t maxval = 100;
        uint16_t n = 1 + (unsigned(random()) % 100);
        uint16_t a1[n];
        uint16_t a2[n];
        for (uint8_t j=0; j<n; j++) {
            a1[j] = a2[j] = unsigned(random()) % maxval;
        }
        insertion_sort_uint16(a1, n);
        qsort(a2, n, sizeof(uint16_t), (__compar_fn_t)comp16);
        check_equal(a1, a2, n);
    }
}

// a dumb version of remove_duplicates_uint16() for testing
static uint16_t dumb_unique(uint16_t *data, uint16_t n)
{
    uint16_t a2[n];
    uint16_t ret = 0;
    a2[0] = data[0];
    for (uint16_t i=1; i<n; i++) {
        if (data[i] != a2[ret]) {
            a2[++ret] = data[i];
        }
    }
    ret++;
    memcpy(data, a2, ret*sizeof(uint16_t));
    return ret;
}

TEST(Sorting, unique)
{
    for (uint16_t i=0; i<10000; i++) {
        const uint8_t maxval = 30;
        uint16_t n = 1 + (unsigned(random()) % 100);
        uint16_t a1[n];
        uint16_t a2[n];
        for (uint8_t j=0; j<n; j++) {
            a1[j] = a2[j] = unsigned(random()) % maxval;
        }
        insertion_sort_uint16(a1, n);
        insertion_sort_uint16(a2, n);
        uint16_t n1 = remove_duplicates_uint16(a1, n);
        uint16_t n2 = dumb_unique(a2, n);
        EXPECT_EQ(n1, n2);
        check_equal(a1, a2, n1);
    }
}

// a dumb version of bisect_search_uint16()
static bool dumb_search(uint16_t *data, uint16_t n, uint16_t value)
{
    for (uint16_t i=0; i<n; i++) {
        if (data[i] == value) {
            return true;
        }
    }
    return false;
}

TEST(Sorting, bisect)
{
    for (uint16_t i=0; i<1000; i++) {
        const uint8_t maxval = 100;
        uint16_t n = 1 + (unsigned(random()) % 100);
        uint16_t a1[n];
        for (uint8_t j=0; j<n; j++) {
            a1[j] = unsigned(random()) % maxval;
        }
        insertion_sort_uint16(a1, n);
        for (uint8_t j=0; j<10; j++) {
            uint16_t v = unsigned(random()) % maxval;
            bool b1 = dumb_search(a1, n, v);
            bool b2 = bisect_search_uint16(a1, n, v);
            EXPECT_EQ(b1, b2);
        }
    }
}

// a dumb version of bisect_search_uint16()
static uint16_t dumb_remove_list(uint16_t *data, uint16_t n, const uint16_t *rem, uint16_t n2)
{
    uint16_t a[n];
    uint16_t ret = 0;
    for (uint16_t i=0; i<n; i++) {
        bool found = false;
        for (uint16_t j=0; j<n2; j++) {
            if (rem[j] == data[i]) {
                found = true;
                break;
            }
        }
        if (!found) {
            a[ret] = data[i];
            ret++;
        }
    }
    memcpy(data, a, ret*sizeof(uint16_t));
    return ret;
}

TEST(Sorting, remove)
{
    for (uint16_t i=0; i<1000; i++) {
        const uint8_t maxval = 100;
        uint16_t n = 1 + (unsigned(random()) % 100);
        uint16_t n2 = 1 + (unsigned(random()) % 100);
        uint16_t a1[n];
        uint16_t a2[n];
        uint16_t a3[n2];
        for (uint8_t j=0; j<n; j++) {
            a2[j] = a1[j] = unsigned(random()) % maxval;
        }
        for (uint8_t j=0; j<n2; j++) {
            a3[j] = unsigned(random()) % maxval;
        }
        insertion_sort_uint16(a1, n);
        insertion_sort_uint16(a2, n);
        insertion_sort_uint16(a3, n2);
        uint16_t r1 = remove_list_uint16(a1, n, a3, n2);
        uint16_t r2 = dumb_remove_list(a2, n, a3, n2);
        EXPECT_EQ(r1, r2);
        for (uint8_t j=0; j<r1; j++) {
            EXPECT_EQ(a1[j], a2[j]);
        }
    }
}

AP_GTEST_MAIN()

#endif // HAL_SITL or HAL_LINUX
