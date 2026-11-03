#include <AP_gtest.h>
#include <AP_JSON/AP_JSON.h>
#include <AP_HAL/AP_HAL.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <type_traits>
#include <utility>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

typedef AP_JSON::value value;

// bit exact comparison, as the parser must return the correctly rounded value
static bool same_double(double a, double b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

static bool parses(const char *json)
{
    value v;
    return AP_JSON::parse(v, std::string(json)).empty();
}

TEST(AP_JSON, ValidDocuments)
{
    EXPECT_TRUE(parses("{}"));
    EXPECT_TRUE(parses("[]"));
    EXPECT_TRUE(parses(" {\"a\" : [1, -2.5, 3e2, 0.1E-3, -0], \"b\": {\"c\": null}} \n"));
    EXPECT_TRUE(parses("{\"t\":true,\"f\":false,\"s\":\"x\\\"y\\\\z\\/\\b\\f\\n\\r\\t\"}"));
}

TEST(AP_JSON, MalformedNumbersRejected)
{
    // all accepted by strtod, none are JSON
    EXPECT_FALSE(parses("{\"a\":01}"));
    EXPECT_FALSE(parses("{\"a\":1.}"));
    EXPECT_FALSE(parses("{\"a\":.5}"));
    EXPECT_FALSE(parses("{\"a\":+1}"));
    EXPECT_FALSE(parses("{\"a\":-}"));
    EXPECT_FALSE(parses("{\"a\":1e}"));
    EXPECT_FALSE(parses("{\"a\":1.e5}"));
    EXPECT_FALSE(parses("{\"a\":1.2.3}"));
    EXPECT_FALSE(parses("{\"a\":NaN}"));
    EXPECT_FALSE(parses("{\"a\":Infinity}"));
}

TEST(AP_JSON, OtherSyntaxErrorsRejected)
{
    EXPECT_FALSE(parses("{\"a\":1,}"));
    EXPECT_FALSE(parses("{\"a\":[1,2,]}"));
    EXPECT_FALSE(parses("{a:1}"));
    EXPECT_FALSE(parses("{'a':1}"));
    EXPECT_FALSE(parses("{\"a\" 1}"));
    EXPECT_FALSE(parses("{\"a\":tru}"));
    EXPECT_FALSE(parses("{\"a\":\"\\x\"}"));
    EXPECT_FALSE(parses("{\"a\":1"));
}

TEST(AP_JSON, TrailingCharactersRejected)
{
    EXPECT_FALSE(parses("{\"a\":1} junk"));
    EXPECT_FALSE(parses("{\"a\":1}{\"b\":2}"));
    EXPECT_TRUE(parses("{\"a\":1} \t\r\n"));
}

TEST(AP_JSON, DuplicateKeysRejected)
{
    EXPECT_FALSE(parses("{\"a\":1,\"a\":2}"));
    EXPECT_FALSE(parses("{\"o\":{\"k\":1,\"b\":2,\"k\":3}}"));
    // the same key in different objects is fine
    EXPECT_TRUE(parses("{\"a\":{\"k\":1},\"b\":{\"k\":2}}"));
}

TEST(AP_JSON, NumbersAreExact)
{
    value v;
    ASSERT_TRUE(AP_JSON::parse(v, std::string("[-7.312715117751976, 1e-7, 48.6493, 12345678901234567890]")).empty());
    const value::array &a = v.get<value::array>();
    ASSERT_EQ(4u, a.size());
    EXPECT_TRUE(same_double(-7.312715117751976, a[0].get<double>()));
    EXPECT_TRUE(same_double(1e-7, a[1].get<double>()));
    EXPECT_TRUE(same_double(48.6493, a[2].get<double>()));
    EXPECT_TRUE(same_double(12345678901234567890.0, a[3].get<double>()));
}

TEST(AP_JSON, LongNumbers)
{
    const std::string n = "1." + std::string(80, '1') + "e-5";
    value v;
    ASSERT_TRUE(AP_JSON::parse(v, "[" + n + "]").empty());
    EXPECT_TRUE(same_double(strtod(n.c_str(), nullptr), v.get(0).get<double>()));
    EXPECT_FALSE(parses(("[1." + std::string(80, '1') + "e]").c_str()));
}

TEST(AP_JSON, UnicodeEscapes)
{
    value v;
    ASSERT_TRUE(AP_JSON::parse(v, std::string("[\"\\u0041\\u00e9\\u20ac\\ud83d\\ude81\"]")).empty());
    // A, e-acute, euro sign, and a 4 byte character from a surrogate pair, as UTF-8
    EXPECT_EQ(std::string("A\xc3\xa9\xe2\x82\xac\xf0\x9f\x9a\x81"), v.get(0).get<std::string>());

    EXPECT_FALSE(parses("[\"\\u12\"]"));
    EXPECT_FALSE(parses("[\"\\ud83d\"]"));        // lone high surrogate
    EXPECT_FALSE(parses("[\"\\ude81\"]"));        // lone low surrogate
    EXPECT_FALSE(parses("[\"\\ud83d\\u0041\"]")); // high surrogate not followed by a low one
}

TEST(AP_JSON, LookupsOnWrongTypeAreSafe)
{
    value v;
    ASSERT_TRUE(AP_JSON::parse(v, std::string("{\"n\":1,\"arr\":[1,2],\"obj\":{\"k\":2}}")).empty());
    const value &n = v.get("n");
    const value &arr = v.get("arr");

    // key lookups on a number or an array return null rather than
    // reading the wrong member of the internal union
    EXPECT_TRUE(n.get("k").is<AP_JSON::null>());
    EXPECT_FALSE(n.contains("k"));
    EXPECT_TRUE(arr.get("k").is<AP_JSON::null>());
    EXPECT_FALSE(arr.contains("k"));

    // index lookups on a number or an object
    EXPECT_TRUE(n.get(0).is<AP_JSON::null>());
    EXPECT_FALSE(n.contains(0));
    EXPECT_TRUE(v.get(0).is<AP_JSON::null>());
    EXPECT_FALSE(v.contains(0));

    // and normal lookups still work
    EXPECT_TRUE(arr.contains(1));
    EXPECT_FALSE(arr.contains(2));
    EXPECT_DOUBLE_EQ(2.0, v.get("obj").get("k").get<double>());
    EXPECT_TRUE(v.get("missing").is<AP_JSON::null>());
}

// a failed lookup returns a shared null, so no lookup may hand it out
// for writing
static_assert(std::is_const<std::remove_reference<decltype(std::declval<value&>().get("k"))>::type>::value,
              "get(key) must return a const reference");
static_assert(std::is_const<std::remove_reference<decltype(std::declval<value&>().get(0))>::type>::value,
              "get(idx) must return a const reference");

static AP_JSON::value *load_json_text(const char *text)
{
    const char *path = "test_ap_json_load.json";
    FILE *f = fopen(path, "w");
    if (f == nullptr) {
        return nullptr;
    }
    fputs(text, f);
    fclose(f);
    AP_JSON::value *v = AP_JSON::load_json(path);
    remove(path);
    return v;
}

TEST(AP_JSON, LoadJsonStripsComments)
{
    // # starts a comment outside strings only, including on the last
    // line with no newline after it
    AP_JSON::value *v = load_json_text("# header\n"
                                       "{\"url\": \"http://x/#frag\", # comment\n"
                                       " \"q\": \"a\\\"#b\", \"n\": 1}\n"
                                       "# end");
    ASSERT_NE(nullptr, v);
    EXPECT_EQ(std::string("http://x/#frag"), v->get("url").get<std::string>());
    EXPECT_EQ(std::string("a\"#b"), v->get("q").get<std::string>());
    EXPECT_DOUBLE_EQ(1.0, v->get("n").get<double>());
    delete v;
}

AP_GTEST_MAIN()
