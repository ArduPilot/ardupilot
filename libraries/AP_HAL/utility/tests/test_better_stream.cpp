/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_gtest.h>

#include <string.h>
#include <cmath>

#include <AP_HAL/utility/BetterStream.h>

/*
  minimal BetterStream implementation capturing all output and
  feeding reads from a caller-supplied buffer.  Used to exercise both
  the non-pure BetterStream methods and print_vprintf behind them.
 */
class CaptureStream : public AP_HAL::BetterStream {
public:
    size_t write(uint8_t b) override {
        if (output_len < sizeof(output)-1) {
            output[output_len++] = b;
            output[output_len] = 0;
        }
        return 1;
    }
    size_t write(const uint8_t *buffer, size_t size) override {
        for (size_t i=0; i<size; i++) {
            write(buffer[i]);
        }
        return size;
    }
    using AP_HAL::BetterStream::write;

    uint32_t available() override { return input_len - input_ofs; }
    bool read(uint8_t &b) override {
        if (input_ofs >= input_len) {
            return false;
        }
        b = input[input_ofs++];
        return true;
    }
    using AP_HAL::BetterStream::read;

    bool discard_input() override {
        input_ofs = input_len;
        return true;
    }
    uint32_t txspace() override { return sizeof(output) - output_len; }

    void set_input(const char *str) {
        strncpy(input, str, sizeof(input)-1);
        input_len = strlen(str);
        input_ofs = 0;
    }
    void reset() {
        output_len = 0;
        output[0] = 0;
        input_len = 0;
        input_ofs = 0;
    }

    char output[512] {};
    size_t output_len;
    char input[64] {};
    size_t input_len;
    size_t input_ofs;
};

static CaptureStream stream;

// printf to a fresh stream and return the captured output
static const char *fmt_result(const char *fmt, ...) FMT_PRINTF(1, 2);
static const char *fmt_result(const char *fmt, ...)
{
    stream.reset();
    va_list ap;
    va_start(ap, fmt);
    stream.vprintf(fmt, ap);
    va_end(ap);
    return stream.output;
}

TEST(BetterStream, PrintfStrings)
{
    EXPECT_STREQ(fmt_result("hello, world"), "hello, world");
    EXPECT_STREQ(fmt_result("%s", "hello"), "hello");
    EXPECT_STREQ(fmt_result("[%10s]", "hello"), "[     hello]");
    EXPECT_STREQ(fmt_result("[%-10s]", "hello"), "[hello     ]");
    EXPECT_STREQ(fmt_result("%.3s", "hello"), "hel");
    EXPECT_STREQ(fmt_result("%c", 'A'), "A");
    EXPECT_STREQ(fmt_result("[%5c]", 'A'), "[    A]");
    EXPECT_STREQ(fmt_result("100%%"), "100%");
}

TEST(BetterStream, PrintfIntegers)
{
    EXPECT_STREQ(fmt_result("%d", 0), "0");
    EXPECT_STREQ(fmt_result("%d", 123), "123");
    EXPECT_STREQ(fmt_result("%d", -123), "-123");
    EXPECT_STREQ(fmt_result("%i", 45), "45");
    EXPECT_STREQ(fmt_result("%+d", 42), "+42");
    EXPECT_STREQ(fmt_result("% d", 42), " 42");
    EXPECT_STREQ(fmt_result("[%5d]", 42), "[   42]");
    EXPECT_STREQ(fmt_result("[%-5d]", 42), "[42   ]");
    EXPECT_STREQ(fmt_result("%05d", 42), "00042");
    EXPECT_STREQ(fmt_result("%.5d", 42), "00042");
    EXPECT_STREQ(fmt_result("%u", 4294967295U), "4294967295");
    EXPECT_STREQ(fmt_result("%ld", -1234567890L), "-1234567890");
    EXPECT_STREQ(fmt_result("%lu", 4000000000LU), "4000000000");
    EXPECT_STREQ(fmt_result("%lld", -1234567890123LL), "-1234567890123");
    // values much beyond INT64_MAX overflow ulltoa_invert's decimal
    // conversion (acknowledged in a comment there), so stick to this:
    EXPECT_STREQ(fmt_result("%llu", 9223372036854775807ULL), "9223372036854775807");
}

TEST(BetterStream, PrintfBases)
{
    // this printf deviates from the standard: hex is always
    // upper-case, whether via %x or %X (XTOA_UPPER is set at compile
    // time in ultoa_invert, and doesn't fit in its uint8_t base
    // parameter anyway)
    EXPECT_STREQ(fmt_result("%x", 0xdeadbeefU), "DEADBEEF");
    EXPECT_STREQ(fmt_result("%X", 0xdeadbeefU), "DEADBEEF");
    EXPECT_STREQ(fmt_result("%#x", 0xbeefU), "0xBEEF");
    EXPECT_STREQ(fmt_result("%#X", 0xbeefU), "0XBEEF");
    EXPECT_STREQ(fmt_result("%o", 8U), "10");
    EXPECT_STREQ(fmt_result("%#o", 8U), "010");
    EXPECT_STREQ(fmt_result("%08x", 0x1fU), "0000001F");
    EXPECT_STREQ(fmt_result("%llx", 0x123456789abcdefULL), "123456789ABCDEF");
    EXPECT_STREQ(fmt_result("%p", (void*)0x1234), "0x1234");
}

TEST(BetterStream, PrintfFloats)
{
    EXPECT_STREQ(fmt_result("%f", 1.5), "1.500000");
    EXPECT_STREQ(fmt_result("%f", 0.0), "0.000000");
    EXPECT_STREQ(fmt_result("%.2f", 3.14159), "3.14");
    EXPECT_STREQ(fmt_result("%.2f", -0.5), "-0.50");
    EXPECT_STREQ(fmt_result("%.0f", 3.0), "3");
    EXPECT_STREQ(fmt_result("%+.1f", 2.5), "+2.5");
    EXPECT_STREQ(fmt_result("[%8.2f]", 3.5), "[    3.50]");
    EXPECT_STREQ(fmt_result("[%-8.2f]", 3.5), "[3.50    ]");
    EXPECT_STREQ(fmt_result("%08.2f", 3.5), "00003.50");
    EXPECT_STREQ(fmt_result("%.3f", 0.125), "0.125");
}

TEST(BetterStream, PrintfFloatExponent)
{
    EXPECT_STREQ(fmt_result("%e", 1234.5), "1.234500e+03");
    EXPECT_STREQ(fmt_result("%E", 1234.5), "1.234500E+03");
    EXPECT_STREQ(fmt_result("%.2e", 0.00012345), "1.23e-04");
    EXPECT_STREQ(fmt_result("%g", 0.0001), "0.0001");
    EXPECT_STREQ(fmt_result("%g", 1234.5), "1234.5");
    EXPECT_STREQ(fmt_result("%g", 100000.0), "100000");
    EXPECT_STREQ(fmt_result("%g", 10000000.0), "1e+07");
    EXPECT_STREQ(fmt_result("%G", 10000000.0), "1E+07");

    // this printf deviates from the standard: %f of anything larger
    // than 9999999 falls back to exponent notation
    EXPECT_STREQ(fmt_result("%f", 100000000.0), "1.000000e+08");
}

TEST(BetterStream, PrintfFloatSpecials)
{
    EXPECT_STREQ(fmt_result("%f", (double)INFINITY), "inf");
    EXPECT_STREQ(fmt_result("%f", (double)-INFINITY), "-inf");
    EXPECT_STREQ(fmt_result("%F", (double)INFINITY), "INF");
    EXPECT_STREQ(fmt_result("%f", (double)NAN), "nan");
    EXPECT_STREQ(fmt_result("%F", (double)NAN), "NAN");
    EXPECT_STREQ(fmt_result("[%6f]", (double)INFINITY), "[   inf]");
    EXPECT_STREQ(fmt_result("%+f", (double)INFINITY), "+inf");
}

TEST(BetterStream, Write)
{
    stream.reset();
    // write(const char*) is implemented in terms of the buffer write
    EXPECT_EQ(stream.write("hello"), 5U);
    EXPECT_STREQ(stream.output, "hello");

    stream.reset();
    stream.print("abc");
    EXPECT_STREQ(stream.output, "abc");

    stream.reset();
    stream.println("abc");
    EXPECT_STREQ(stream.output, "abc\r\n");
}

TEST(BetterStream, Read)
{
    stream.reset();
    // read of an empty stream fails
    EXPECT_EQ(stream.read(), -1);

    stream.set_input("abcde");
    EXPECT_EQ(stream.available(), 5U);
    // single-byte read
    EXPECT_EQ(stream.read(), 'a');

    // multi-byte read
    uint8_t buf[16] {};
    EXPECT_EQ(stream.read(buf, 2), 2);
    EXPECT_EQ(buf[0], 'b');
    EXPECT_EQ(buf[1], 'c');

    // a read larger than available returns only what is there
    EXPECT_EQ(stream.read(buf, sizeof(buf)), 2);
    EXPECT_EQ(buf[0], 'd');
    EXPECT_EQ(buf[1], 'e');
    EXPECT_EQ(stream.available(), 0U);

    stream.set_input("xyz");
    EXPECT_TRUE(stream.discard_input());
    EXPECT_EQ(stream.available(), 0U);
    EXPECT_EQ(stream.read(), -1);
}

AP_GTEST_MAIN()
