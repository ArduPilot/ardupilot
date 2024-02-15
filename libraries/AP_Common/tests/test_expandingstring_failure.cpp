#include <AP_gtest.h>
#include <stdlib.h>
#include <AP_Common/ExpandingString.h>
#include <AP_HAL/AP_HAL.h>

/**
 * This file test realloc failure on ExpandingString
 */

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static uint32_t count = 0;

void *realloc(void *ptr, size_t new_size) {
    count++;
    if (count < 3) {
        if (new_size == 0) {
            free(ptr);
            return nullptr;
        }
        if (ptr == nullptr) {
            return malloc(new_size);
        }
        void *new_mem = malloc(new_size);
        if (new_mem != nullptr) {
            memcpy(new_mem, ptr, new_size);
            free(ptr);
        }
        return new_mem;
    } else {
        return nullptr;
    }
}


// THAT IS UGLY HACK BUT IT WORKS ... it is just used to make print_vprintf return negative value.
class BufferPrinter : public AP_HAL::BetterStream {
public:
    BufferPrinter(char* str, size_t size)  :
    _offs(0), _str(str), _size(size)  {}

    size_t write(uint8_t c) override { return 1; }
    size_t write(const uint8_t *buffer, size_t size) override { return 1; }

    size_t _offs;
    char* const  _str;
    const size_t _size;

    uint32_t available() override { return 0; }
    bool read(uint8_t &b) override { return false; }
    uint32_t txspace() override { return 0; }
    bool discard_input() override { return false; }
};

void print_vprintf(AP_HAL::BetterStream *s, const char *fmt, va_list ap);
void print_vprintf(AP_HAL::BetterStream *s, const char *fmt, va_list ap) {
    BufferPrinter* p = static_cast<BufferPrinter*>(s);
    if (count < 2) {
        p->_offs = -1;
        return;
    }
    if (count == 2) {
        p->_offs = p->_size * 2;
    } else {
        p->_offs = p->_size;
    }
    return;
}

TEST(ExpandingString, Tests)
{
    // Test print_vprintf failure.
    ExpandingString *test_string = new ExpandingString();
    test_string->printf("Test\n");
    EXPECT_STREQ("", test_string->get_string());
    EXPECT_STREQ("", test_string->get_writeable_string());
    EXPECT_EQ(0u, test_string->get_length());
    EXPECT_FALSE(test_string->has_failed_allocation());
    // test failure on second printf expand()
    test_string = new ExpandingString();
    test_string->printf("Test\n");
    EXPECT_STREQ("", test_string->get_string());
    EXPECT_STREQ("", test_string->get_writeable_string());
    EXPECT_EQ(0u, test_string->get_length());
    EXPECT_TRUE(test_string->has_failed_allocation());
    // Test realloc failure
    test_string = new ExpandingString();
    test_string->printf("Test\n");
    EXPECT_STREQ(nullptr, test_string->get_string());
    EXPECT_STREQ(nullptr, test_string->get_writeable_string());
    EXPECT_EQ(0u, test_string->get_length());
    EXPECT_TRUE(test_string->has_failed_allocation());
    // test append failure
    EXPECT_FALSE(test_string->append("Test2\n", 6));
    // test failure on first printf realloc
    test_string->printf("Test\n");
    EXPECT_STREQ(nullptr, test_string->get_string());
    EXPECT_STREQ(nullptr, test_string->get_writeable_string());
    EXPECT_EQ(0u, test_string->get_length());
    EXPECT_TRUE(test_string->has_failed_allocation());
    // test failure on append realloc
    test_string = new ExpandingString();
    EXPECT_FALSE(test_string->append("Test2\n", 6));
    EXPECT_TRUE(test_string->has_failed_allocation());
    EXPECT_STREQ(nullptr, test_string->get_string());
    EXPECT_EQ(0u, test_string->get_length());

    test_string->~ExpandingString();
    EXPECT_STRNE("Test\n", test_string->get_string());
}

TEST(ExpandingString, TestsFailure)
{


}
AP_GTEST_MAIN()
