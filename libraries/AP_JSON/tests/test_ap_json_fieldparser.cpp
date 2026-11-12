#include <AP_gtest.h>
#include <AP_JSON/AP_JSON_FieldParser.h>
#include <AP_JSON/AP_JSON_Number.h>
#include <AP_Math/crc.h>
#include <AP_HAL/AP_HAL.h>

#include <algorithm>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <string>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

typedef AP_JSON_FieldParser::Type T;
typedef AP_JSON_FieldParser::Result R;

static struct {
    double timestamp;
    float gyro[3];
    double position[3];
    bool flag;
    float rc1;
    float deep;
} st;

static const AP_JSON_FieldParser::Field fields[] = {
    { "timestamp", "t", T::DOUBLE, 1, &st.timestamp },
    { "imu.gyro", "g", T::FLOAT_ARRAY, 3, st.gyro },
    { "position", nullptr, T::DOUBLE_ARRAY, 3, st.position },
    { "flag", nullptr, T::BOOL, 1, &st.flag },
    { "rc.rc_1", "c1", T::FLOAT, 1, &st.rc1 },
    { "a.b.c", nullptr, T::FLOAT, 1, &st.deep },
};
static const uint64_t TIMESTAMP = 1, GYRO = 2, POSITION = 4, FLAG = 8, RC1 = 16, DEEP = 32;

// bit exact comparison
static bool same(double a, double b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}
static bool same_f(float a, float b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

// feed a string, returning the results that are not NONE in order
static std::string feed(AP_JSON_FieldParser &p, const std::string &s)
{
    std::string results;
    for (char c : s) {
        switch (p.feed(uint8_t(c))) {
        case R::NONE:
            break;
        case R::PACKET:
            results += 'P';
            break;
        case R::ERROR:
            results += 'E';
            break;
        }
    }
    return results;
}

static bool accepted(const std::string &s)
{
    AP_JSON_FieldParser p(fields, 6);
    return feed(p, s) == "P";
}

TEST(AP_JSON_FieldParser, FullPathsAndValues)
{
    memset(&st, 0, sizeof(st));
    AP_JSON_FieldParser p(fields, 6);
    EXPECT_EQ("P", feed(p, "{\"timestamp\":12.5,\"imu\":{\"gyro\":[0.1,-0.2,3e-3],\"accel\":[1,2,3]},"
                           "\"position\":[1.5,-2,3],\"flag\":true,\"rc\":{\"rc_1\":1500},\"a\":{\"b\":{\"c\":7}},"
                           "\"other\":[{\"x\":null},\"s\",false]}\n"));
    EXPECT_EQ(TIMESTAMP | GYRO | POSITION | FLAG | RC1 | DEEP, p.found());
    EXPECT_FALSE(p.had_crc());
    EXPECT_TRUE(same(12.5, st.timestamp));
    EXPECT_TRUE(same_f(strtof("0.1", nullptr), st.gyro[0]));
    EXPECT_TRUE(same_f(strtof("-0.2", nullptr), st.gyro[1]));
    EXPECT_TRUE(same_f(strtof("3e-3", nullptr), st.gyro[2]));
    EXPECT_TRUE(same(-2.0, st.position[1]));
    EXPECT_TRUE(st.flag);
    EXPECT_TRUE(same_f(strtof("1500", nullptr), st.rc1));
    EXPECT_TRUE(same_f(strtof("7", nullptr), st.deep));
}

TEST(AP_JSON_FieldParser, OnlyPresentFieldsAreFound)
{
    AP_JSON_FieldParser p(fields, 6);
    EXPECT_EQ("P", feed(p, "{\"timestamp\":1}\n"));
    EXPECT_EQ(TIMESTAMP, p.found());
    EXPECT_EQ("P", feed(p, "{}\n"));
    EXPECT_EQ(0U, p.found());
}

TEST(AP_JSON_FieldParser, KeysMatchOnlyAtTheirLocation)
{
    memset(&st, 0, sizeof(st));
    AP_JSON_FieldParser p(fields, 6);
    // gyro outside imu, rc_1 at the root, timestamp inside an object or
    // an array, a key name inside a string: none of these match
    EXPECT_EQ("P", feed(p, "{\"gyro\":[1,2,3],\"rc_1\":5,\"x\":{\"timestamp\":3},\"y\":[{\"timestamp\":4}],"
                           "\"z\":\"timestamp\",\"imu\":{\"x\":{\"gyro\":[1,2,3]}}}\n"));
    EXPECT_EQ(0U, p.found());
    // keys containing a dot, an escape or longer than MAX_KEY_LEN never match
    EXPECT_EQ("P", feed(p, "{\"imu.gyro\":[1,2,3],\"t\\u0069mestamp\":1,\"" + std::string(40, 'k') + "\":1}\n"));
    EXPECT_EQ(0U, p.found());
}

TEST(AP_JSON_FieldParser, Aliases)
{
    memset(&st, 0, sizeof(st));
    AP_JSON_FieldParser p(fields, 6);
    EXPECT_EQ("P", feed(p, "{\"t\":2.5,\"g\":[1,2,3],\"c1\":1100}\n"));
    EXPECT_EQ(TIMESTAMP | GYRO | RC1, p.found());
    EXPECT_TRUE(same(2.5, st.timestamp));

    // the full path wins whatever the order
    EXPECT_EQ("P", feed(p, "{\"t\":1,\"timestamp\":2}\n"));
    EXPECT_TRUE(same(2.0, st.timestamp));
    EXPECT_EQ("P", feed(p, "{\"timestamp\":3,\"t\":4}\n"));
    EXPECT_TRUE(same(3.0, st.timestamp));

    // an alias is only a key of the root object
    EXPECT_EQ("P", feed(p, "{\"rc\":{\"c1\":5}}\n"));
    EXPECT_EQ(0U, p.found());

    // a discarded alias is still type checked, so key order does not matter
    EXPECT_FALSE(accepted("{\"timestamp\":3,\"t\":\"x\"}\n"));
    EXPECT_FALSE(accepted("{\"t\":\"x\",\"timestamp\":3}\n"));
}

TEST(AP_JSON_FieldParser, DuplicateKeysRejected)
{
    // a field, an alias or an object holding fields given twice, as
    // AP_JSON rejects duplicate keys
    EXPECT_FALSE(accepted("{\"timestamp\":1,\"timestamp\":2}\n"));
    EXPECT_FALSE(accepted("{\"t\":1,\"t\":2}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2,3],\"gyro\":[1,2,3]}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2,3]},\"imu\":{}}\n"));
    EXPECT_FALSE(accepted("{\"a\":{\"b\":{}},\"a\":5}\n"));
    EXPECT_FALSE(accepted("{\"a\":{\"b\":{},\"b\":{\"c\":1}}}\n"));

    // the same key in different objects, and a field with its alias
    EXPECT_TRUE(accepted("{\"imu\":{\"gyro\":[1,2,3]},\"x\":{\"imu\":1},\"y\":[{\"imu\":1},{\"imu\":2}]}\n"));
    EXPECT_TRUE(accepted("{\"timestamp\":1,\"t\":2,\"g\":[1,2,3],\"imu\":{\"gyro\":[1,2,3]}}\n"));

    // keys outside the field table are not checked
    EXPECT_TRUE(accepted("{\"other\":1,\"other\":2}\n"));

    // a rejected packet does not affect the next one
    AP_JSON_FieldParser p(fields, 6);
    EXPECT_EQ("EP", feed(p, "{\"t\":1,\"t\":2}\n{\"t\":1}\n"));
    EXPECT_EQ(TIMESTAMP, p.found());
}

TEST(AP_JSON_FieldParser, WrongTypesRejected)
{
    EXPECT_FALSE(accepted("{\"timestamp\":\"1\"}\n"));
    EXPECT_FALSE(accepted("{\"timestamp\":[1]}\n"));
    EXPECT_FALSE(accepted("{\"timestamp\":{}}\n"));
    EXPECT_FALSE(accepted("{\"timestamp\":true}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2]}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2,3,4]}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2,\"3\"]}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2,[3]]}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":5}}\n"));
    EXPECT_FALSE(accepted("{\"flag\":null}\n"));
    EXPECT_FALSE(accepted("{\"flag\":NaN}\n"));
    EXPECT_FALSE(accepted("{\"flag\":-inf}\n"));
    EXPECT_FALSE(accepted("{\"flag\":\"true\"}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":null}}\n"));
    EXPECT_FALSE(accepted("{\"imu\":{\"gyro\":[1,2,true]}}\n"));
    // a section that is not an object is simply not looked into
    EXPECT_TRUE(accepted("{\"imu\":5,\"rc\":[1,2]}\n"));
}

TEST(AP_JSON_FieldParser, Booleans)
{
    AP_JSON_FieldParser p(fields, 6);
    const char *cases[][2] = {
        { "true", "1" }, { "false", "0" }, { "1", "1" }, { "0", "0" }, { "-0.0", "0" }, { "2.5", "1" },
    };
    for (const auto &c : cases) {
        st.flag = !(c[1][0] == '1');
        EXPECT_EQ("P", feed(p, std::string("{\"flag\":") + c[0] + "}\n")) << c[0];
        EXPECT_EQ(c[1][0] == '1', st.flag) << c[0];
    }
}

TEST(AP_JSON_FieldParser, StrictJson)
{
    const char *bad[] = {
        "{\"timestamp\":01}", "{\"timestamp\":-01}", "{\"timestamp\":-00.5}", "{\"x\":-01}",
        "{\"timestamp\":1.}", "{\"timestamp\":.5}", "{\"timestamp\":+1}",
        "{\"timestamp\":-}", "{\"timestamp\":1e}", "{\"timestamp\":1,}",
        "{\"x\":[1,2,]}", "{x:1}", "{'x':1}", "{\"x\" 1}", "{\"x\":tru}", "{\"x\":nul}",
        "{\"x\":\"\\q\"}", "{\"x\":\"\\u12\"}", "{\"x\":\"\\ud83d\"}", "{\"x\":\"\\ude81\"}",
        "{\"x\":\"\\ud83d\\u0041\"}", "{\"x\":\"a\x01\"}", "{\"x\":1]", "{\"x\":[1}", "{\"x\":1}}",
        "{\"x\":1} junk", "{\"x\":1}{\"y\":2}", "[1,2]", "\"x\"", "1",
    };
    for (const char *b : bad) {
        EXPECT_FALSE(accepted(std::string(b) + "\n")) << b;
    }
    const char *good[] = {
        "{}", " \t{ } \r", "{\"x\":[]}", "{\"x\":{}}", "{\"x\":\"\\\"\\\\\\/\\b\\f\\n\\r\\t\\u00e9\\ud83d\\ude81\"}",
        "{\"x\":[-0,0.5,1e5,1E-5,1e+5,-1.5e-300,123456789012345678901234567890]}",
        "{\"x\":[true,false,null,\"\",{\"y\":[[]]}]}",
    };
    for (const char *g : good) {
        EXPECT_TRUE(accepted(std::string(g) + "\n")) << g;
    }
}

TEST(AP_JSON_FieldParser, NonFiniteNumbers)
{
    // JSON has no infinity or NaN; these are how encoders write them
    // anyway: null by most (rosbridge, JavaScript, serde_json, MATLAB),
    // NaN / Infinity by Python's json, nan / inf by C's printf
    const char *nan_words[] = { "null", "NaN", "-NaN", "nan", "-nan" };
    const char *inf_words[] = { "Infinity", "inf" };
    AP_JSON_FieldParser p(fields, 6);
    for (const char *w : nan_words) {
        st.timestamp = 0;
        st.rc1 = 0;
        EXPECT_EQ("P", feed(p, std::string("{\"timestamp\":") + w + ",\"c1\":" + w + "}\n")) << w;
        EXPECT_EQ(TIMESTAMP | RC1, p.found()) << w;
        EXPECT_TRUE(isnan(st.timestamp)) << w;
        EXPECT_TRUE(isnan(st.rc1)) << w;
    }
    for (const char *w : inf_words) {
        EXPECT_EQ("P", feed(p, std::string("{\"timestamp\":") + w + ",\"c1\":-" + w + "}\n")) << w;
        EXPECT_TRUE(isinf(st.timestamp) && st.timestamp > 0) << w;
        EXPECT_TRUE(isinf(st.rc1) && st.rc1 < 0) << w;
    }

    // in array fields, and in values that are skipped
    EXPECT_EQ("P", feed(p, "{\"imu\":{\"gyro\":[null,Infinity,-inf]},\"position\":[NaN,1,-Infinity],"
                           "\"other\":[nan,-nan,inf,{\"x\":-Infinity}]}\n"));
    EXPECT_EQ(GYRO | POSITION, p.found());
    EXPECT_TRUE(isnan(st.gyro[0]));
    EXPECT_TRUE(isinf(st.gyro[1]) && st.gyro[1] > 0);
    EXPECT_TRUE(isinf(st.gyro[2]) && st.gyro[2] < 0);
    EXPECT_TRUE(isnan(st.position[0]));
    EXPECT_TRUE(same(1.0, st.position[1]));
    EXPECT_TRUE(isinf(st.position[2]) && st.position[2] < 0);

    // other spellings, and words cut short or run on, are still invalid
    const char *bad[] = {
        "-null", "nul", "nanx", "nAn", "NAN", "-Nan", "Inf", "INF", "infinity", "Infinit",
        "- inf", "--inf", "inf1", "1inf", "-", "-true",
    };
    for (const char *b : bad) {
        EXPECT_FALSE(accepted(std::string("{\"timestamp\":") + b + "}\n")) << b;
        EXPECT_FALSE(accepted(std::string("{\"x\":") + b + "}\n")) << b;
    }
}

TEST(AP_JSON_FieldParser, Framing)
{
    AP_JSON_FieldParser p(fields, 6);
    // nothing until the newline that ends a packet
    EXPECT_EQ("", feed(p, "{\"timestamp\":1}"));
    EXPECT_EQ("P", feed(p, "\n"));
    // blank lines and leading whitespace between packets are fine
    EXPECT_EQ("PP", feed(p, "\n\n  {\"timestamp\":1}\r\n\t{\"timestamp\":2}\n"));
    // a packet split at every byte, as from a UART
    std::string results;
    for (char c : std::string("{\"imu\":{\"gyro\":[1,2,3]}}\n")) {
        results += feed(p, std::string(1, c));
    }
    EXPECT_EQ("P", results);
    EXPECT_EQ(GYRO, p.found());
}

TEST(AP_JSON_FieldParser, ResyncAfterErrors)
{
    AP_JSON_FieldParser p(fields, 6);
    // garbage, a truncated packet and a bad packet: each gives one error
    // and the next line parses
    EXPECT_EQ("EP", feed(p, "garbage {\"timestamp\":1}\n{\"timestamp\":2}\n"));
    EXPECT_EQ("EP", feed(p, "{\"timestamp\":1,\"imu\":{\"gy\n{\"timestamp\":3}\n"));
    EXPECT_EQ("EP", feed(p, "{\"timestamp\":01}\n{\"timestamp\":4}\n"));
    EXPECT_TRUE(same(4.0, st.timestamp));
    // a newline inside a packet ends it, and what is left of it on the
    // next line is an error of its own
    EXPECT_EQ("EEP", feed(p, "{\"timestamp\":\n1}\n{\"timestamp\":5}\n"));
}

TEST(AP_JSON_FieldParser, NulEndsALine)
{
    const std::string nul(1, '\0');
    AP_JSON_FieldParser p(fields, 6);
    // as sent by a C sender writing strlen + 1 bytes
    EXPECT_EQ("P", feed(p, "{\"timestamp\":1}" + nul));
    EXPECT_TRUE(same(1.0, st.timestamp));
    EXPECT_EQ("PP", feed(p, "{\"timestamp\":2}\n" + nul + nul + "{\"timestamp\":3}" + nul));
    // inside a packet it ends the packet like a newline would, and what
    // is left of the line is an error of its own
    EXPECT_EQ("EEP", feed(p, "{\"timestamp\":" + nul + "1}\n{\"timestamp\":4}\n"));
    EXPECT_TRUE(same(4.0, st.timestamp));
}

TEST(AP_JSON_FieldParser, MessageBoundaries)
{
    AP_JSON_FieldParser p(fields, 6);
    // a message ending after a complete object ends the packet
    EXPECT_EQ("", feed(p, "{\"timestamp\":1}"));
    EXPECT_TRUE(p.end_of_message() == R::PACKET);
    EXPECT_TRUE(same(1.0, st.timestamp));

    // a CRC before the end of the message is still checked
    const char *body = "{\"timestamp\":2}";
    const uint16_t crc = crc16_ccitt((const uint8_t *)body, strlen(body), 0xFFFF);
    char good[40], bad[40];
    snprintf(good, sizeof(good), "%s*%04X", body, unsigned(crc));
    snprintf(bad, sizeof(bad), "%s*%04X", body, unsigned(crc ^ 1));
    EXPECT_EQ("", feed(p, good));
    EXPECT_TRUE(p.end_of_message() == R::PACKET);
    EXPECT_TRUE(p.had_crc());
    EXPECT_EQ("", feed(p, bad));
    EXPECT_TRUE(p.end_of_message() == R::ERROR);

    // nothing happens at the end of a message after a newline or between packets
    EXPECT_EQ("P", feed(p, "{\"timestamp\":3}\n"));
    EXPECT_TRUE(p.end_of_message() == R::NONE);
    EXPECT_TRUE(p.end_of_message() == R::NONE);

    // a packet part way through is invalid, and does not cost the packet
    // in the next message
    EXPECT_EQ("", feed(p, "{\"timestamp\":"));
    EXPECT_TRUE(p.end_of_message() == R::ERROR);
    EXPECT_EQ("P", feed(p, "{\"timestamp\":4}\n"));
    EXPECT_TRUE(same(4.0, st.timestamp));

    // garbage in one message does not cost the packet in the next
    EXPECT_EQ("E", feed(p, "hello"));
    EXPECT_TRUE(p.end_of_message() == R::NONE);
    EXPECT_EQ("P", feed(p, "{\"timestamp\":5}\n"));

    // whereas on a stream the parser skips to the next end of line
    EXPECT_EQ("E", feed(p, "hello"));
    EXPECT_EQ("", feed(p, "{\"timestamp\":6}\n"));
    EXPECT_EQ("P", feed(p, "{\"timestamp\":7}\n"));
}

TEST(AP_JSON_FieldParser, Crc)
{
    AP_JSON_FieldParser p(fields, 6);
    // CRC-16/CCITT-FALSE of {"timestamp":1}, as binascii.crc_hqx(b'{"timestamp":1}', 0xFFFF)
    const char *body = "{\"timestamp\":1}";
    const uint16_t crc = crc16_ccitt((const uint8_t *)body, strlen(body), 0xFFFF);
    char good[40], bad[40], lower[40];
    snprintf(good, sizeof(good), "%s*%04X\n", body, unsigned(crc));
    snprintf(bad, sizeof(bad), "%s*%04X\n", body, unsigned(crc ^ 1));
    snprintf(lower, sizeof(lower), "%s *%04x \r\n", body, unsigned(crc));
    EXPECT_EQ("P", feed(p, good));
    EXPECT_TRUE(p.had_crc());
    EXPECT_EQ("P", feed(p, lower));
    EXPECT_EQ("E", feed(p, bad));
    EXPECT_STREQ("CRC mismatch", p.error());
    // a changed digit is caught
    std::string changed = good;
    changed[13] = '2';
    EXPECT_EQ("E", feed(p, changed));
    EXPECT_EQ("E", feed(p, std::string(body) + "*12G4\n"));
    EXPECT_EQ("E", feed(p, std::string(body) + "*123\n"));
    EXPECT_EQ("P", feed(p, std::string(body) + "\n"));
    EXPECT_FALSE(p.had_crc());
}

// feed a stream in chunks of 1 to max_chunk bytes, as a UART or USB
// driver hands over whatever has arrived, recording each result with
// the fields and values it left. max_chunk 0 feeds it in one call
static std::string feed_in_chunks(const std::string &stream, uint32_t seed, size_t max_chunk)
{
    AP_JSON_FieldParser p(fields, 6);
    memset(&st, 0, sizeof(st));
    std::string results;
    size_t off = 0;
    while (off < stream.size()) {
        size_t n = stream.size() - off;
        if (max_chunk != 0) {
            seed = seed * 1103515245U + 12345U;
            n = std::min(n, size_t(1 + (seed >> 16) % max_chunk));
        }
        const size_t end = off + n;
        while (off < end) {
            size_t used;
            const R r = p.feed((const uint8_t *)stream.data() + off, end - off, used);
            off += used;
            char buf[160];
            if (r == R::PACKET) {
                snprintf(buf, sizeof(buf), "P %llx %d %.17g %a %a %a %d %a\n", (unsigned long long)p.found(),
                         int(p.had_crc()), st.timestamp, double(st.gyro[0]), st.position[2], double(st.rc1),
                         int(st.flag), double(st.deep));
                results += buf;
            } else if (r == R::ERROR) {
                results += std::string("E ") + p.error() + "\n";
            }
        }
    }
    return results;
}

TEST(AP_JSON_FieldParser, ChunkBoundaries)
{
    // packets of every kind, including some broken ones, back to back
    const char *bodies[] = {
        "{\"timestamp\":1.5,\"imu\":{\"gyro\":[0.25,-1e-3,7]},\"position\":[1,2,3.0625]}",
        "{\"t\":2,\"g\":[1,2,3],\"c1\":1500,\"flag\":true,\"note\":\"esc\\\"ap\\u00e9d \\ud83d\\ude81\"}",
        "{\"timestamp\":3,\"rc\":{\"rc_1\":-Infinity},\"position\":[null,NaN,-inf],\"a\":{\"b\":{\"c\":-0.5e-2}}}",
        "{\"timestamp\":4,\"x\":[[[]],{},\"\",true,false,null,123456789012345678901234567890]}",
        "{\"timestamp\":5,\"imu\":{\"gyro\":[1,2]}}",               // wrong length
        "{\"timestamp\":6,\"timestamp\":7}",                         // duplicate key
        "{\"timestamp\":0.1e1, \"flag\" : 0 }",
    };
    std::string stream;
    for (int k = 0; k < 40; k++) {
        const char *b = bodies[k % ARRAY_SIZE(bodies)];
        stream += b;
        const uint16_t crc = crc16_ccitt((const uint8_t *)b, strlen(b), 0xFFFF);
        char suffix[8];
        switch (k % 5) {
        case 1:
            snprintf(suffix, sizeof(suffix), "*%04X", unsigned(crc));
            stream += suffix;
            break;
        case 3:
            snprintf(suffix, sizeof(suffix), "*%04X", unsigned(crc ^ 0x0100));
            stream += suffix;
            break;
        }
        stream += (k % 4 == 2) ? std::string(1, '\0') : (k % 6 == 5) ? std::string("\r\n\n  ") : std::string("\n");
    }
    stream += "{\"timestamp\":9";   // cut short: no result

    const std::string whole = feed_in_chunks(stream, 0, 0);
    EXPECT_NE(std::string::npos, whole.find("E CRC mismatch"));
    EXPECT_NE(std::string::npos, whole.find("E duplicate key"));
    // one result per complete packet, none for the last one
    EXPECT_EQ(40, std::count(whole.begin(), whole.end(), '\n'));
    for (const size_t max_chunk : { 1, 2, 3, 7, 16, 64 }) {
        for (uint32_t seed = 1; seed <= 25; seed++) {
            EXPECT_EQ(whole, feed_in_chunks(stream, seed, max_chunk)) << "max chunk " << max_chunk << " seed " << seed;
        }
    }
}

TEST(AP_JSON_FieldParser, Limits)
{
    std::string deep = "{\"x\":";
    for (int i = 0; i < 6; i++) {
        deep += "[";
    }
    std::string close(6, ']');
    // root object plus 7 levels is the maximum
    EXPECT_TRUE(accepted(deep + "[]" + close + "}\n"));
    EXPECT_FALSE(accepted(deep + "[[]]" + close + "}\n"));
    // a long string is fine, a packet over MAX_PACKET_LEN is not
    EXPECT_TRUE(accepted("{\"x\":\"" + std::string(4000, 'y') + "\"}\n"));
    EXPECT_FALSE(accepted("{\"x\":\"" + std::string(5000, 'y') + "\"}\n"));
    // a field value longer than the number buffer
    EXPECT_FALSE(accepted("{\"timestamp\":1." + std::string(60, '1') + "}\n"));
    // but a skipped one is only validated
    EXPECT_TRUE(accepted("{\"x\":1." + std::string(60, '1') + "}\n"));
}

TEST(AP_JSON_FieldParser, NumbersMatchStrtodAndStrtof)
{
    uint32_t seed = 1;
    const auto next = [&seed]() {
        seed = seed * 1664525U + 1013904223U;
        return seed >> 8;
    };
    for (uint32_t i = 0; i < 100000; i++) {
        const uint32_t ndigits = 1 + next() % 17;
        std::string digits(1, char('1' + next() % 9));
        while (digits.size() < ndigits) {
            digits += char('0' + next() % 10);
        }
        const uint32_t decimals = next() % ndigits;
        std::string n = (next() & 1) ? "-" : "";
        n += digits.substr(0, ndigits - decimals);
        if (decimals > 0) {
            n += "." + digits.substr(ndigits - decimals);
        }
        if (next() & 1) {
            n += "e" + std::to_string(int(next() % 61) - 30);
        }
        ASSERT_TRUE(same(strtod(n.c_str(), nullptr), AP_JSON_Number::to_double(n.c_str()))) << n;
        ASSERT_TRUE(same_f(strtof(n.c_str(), nullptr), AP_JSON_Number::to_float(n.c_str()))) << n;
    }
    const char *edge[] = { "0", "-0", "0.0", "1e22", "1e23", "1e10", "1e11", "1e-10", "1e-11", "9999999",
                           "16777217", "3.4028235e38", "1e39", "1e-46", "4.9e-324", "1e400" };
    for (const char *n : edge) {
        EXPECT_TRUE(same(strtod(n, nullptr), AP_JSON_Number::to_double(n))) << n;
        EXPECT_TRUE(same_f(strtof(n, nullptr), AP_JSON_Number::to_float(n))) << n;
    }
    // leading zeros offsetting a large exponent: 1e90000 is infinite
    const std::string offset = "0." + std::string(9999, '0') + "1e100000";
    EXPECT_TRUE(same(strtod(offset.c_str(), nullptr), AP_JSON_Number::to_double(offset.c_str())));
    EXPECT_TRUE(same_f(strtof(offset.c_str(), nullptr), AP_JSON_Number::to_float(offset.c_str())));
}

AP_GTEST_MAIN()
