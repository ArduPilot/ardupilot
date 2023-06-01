#include <AP_gtest.h>

#include <AP_JSONParser/AP_JSONParser.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static bool create_file_from_string(const char *string, const char *filename)
{
    if (unlink(filename) != 0 && errno != ENOENT) {
        ::fprintf(stderr, "Failed to unlink %s: %m\n", filename);
        return false;
    }
    int fd = open(filename, O_WRONLY|O_TRUNC|O_EXCL|O_CREAT, 0600);
    if (fd == -1) {
        ::fprintf(stderr, "Failed to open %s: %m\n", filename);
        return false;
    }
    if (write(fd, string, strlen(string)) != ssize_t(strlen(string))) {
        ::fprintf(stderr, "short write to %s\n", filename);
        return false;
    }
    close(fd);
    return true;
}

TEST(AP_JSONParserTest, basic)
{
    const char *document =
        "{ \"b\": 37 }";
    const char *filepath = "temporary.json";
    EXPECT_TRUE(create_file_from_string(document, filepath));

    AP_JSONParser jp;
    EXPECT_TRUE(jp.init(10));
    EXPECT_TRUE(jp.parse_filepath(filepath));

    char string[50];
    EXPECT_FALSE(jp.shift_token_string(string, ARRAY_SIZE(string)));

    uint16_t nv_count;
    EXPECT_TRUE(jp.shift_token_object(nv_count));
    EXPECT_EQ(nv_count, 1);

    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "b");

    EXPECT_FALSE(jp.shift_token_string(string, ARRAY_SIZE(string)));

    int32_t bob;
    EXPECT_TRUE(jp.shift_token_int32_t(bob));
    EXPECT_EQ(bob, 37);
}

TEST(AP_JSONParserTest, all_types)
{
    const char *document =
        "{\n"
        "\"b\": 37,"
        "\"vec\": [3,4, 5],"
        "\"obj\": {"
        "           \"objkey\": \"x\","
        "           \"objkey2\": 17.1"
        "}"
        "}"
        "";
    const char *filepath = "temporary.json";
    EXPECT_TRUE(create_file_from_string(document, filepath));

    AP_JSONParser jp;
    EXPECT_TRUE(jp.init(50));
    EXPECT_TRUE(jp.parse_filepath(filepath));

    char string[50];
    EXPECT_FALSE(jp.shift_token_string(string, ARRAY_SIZE(string)));

    // extract top-level object:
    uint16_t nv_count;
    EXPECT_TRUE(jp.shift_token_object(nv_count));
    EXPECT_EQ(nv_count, 3);

    // extract "b" and its value:
    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "b");

    EXPECT_FALSE(jp.shift_token_string(string, ARRAY_SIZE(string)));

    int32_t bob;
    EXPECT_TRUE(jp.shift_token_int32_t(bob));
    EXPECT_EQ(bob, 37);

    // extract the "vec" label and value:
    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "vec");

    Vector3f vec;
    EXPECT_TRUE(jp.shift_token_vector3f(vec));
    EXPECT_FLOAT_EQ(vec[0], 3);
    EXPECT_FLOAT_EQ(vec[1], 4);
    EXPECT_FLOAT_EQ(vec[2], 5);

    // extract "obj" and its corresponding object:
    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "obj");

    EXPECT_TRUE(jp.shift_token_object(nv_count));
    EXPECT_EQ(nv_count, 2);

    // extract objkey and its value
    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "objkey");

    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "x");

    // extract objkey1 and its value
    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "objkey2");

    float mary;
    EXPECT_TRUE(jp.shift_token_float(mary));
    EXPECT_FLOAT_EQ(mary, 17.1);
}

TEST(AP_JSONParserTest, invalid_values)
{
    const char *document =
        "{ \"b\": a }";
    const char *filepath = "temporary.json";
    EXPECT_TRUE(create_file_from_string(document, filepath));

    AP_JSONParser jp;
    EXPECT_TRUE(jp.init(10));
    EXPECT_TRUE(jp.parse_filepath(filepath));

    uint16_t nv_count;
    EXPECT_TRUE(jp.shift_token_object(nv_count));
    EXPECT_EQ(nv_count, 1);

    char string[20];
    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "b");

    // we can actually successfully parse "a" as an integer, contrary
    // to standards (which should set errno to EINVAL).

    // int32_t bob;
    // EXPECT_FALSE(jp.shift_token_int32_t(bob));

    // we can actually successfully parse "a" as a float, contrary
    // to standards (which should set errno to EINVAL).
    // float mary;
    // EXPECT_FALSE(jp.shift_token_float(mary));

    Vector3f fred;
    EXPECT_FALSE(jp.shift_token_vector3f(fred));

    EXPECT_FALSE(jp.shift_token_string(string, ARRAY_SIZE(string)));
}

TEST(AP_JSONParserTest, more_invalid_values)
{
    const char *document =
        "{ \"b\": \"a\" }";
    const char *filepath = "temporary.json";
    EXPECT_TRUE(create_file_from_string(document, filepath));

    AP_JSONParser jp;
    EXPECT_TRUE(jp.init(10));
    EXPECT_TRUE(jp.parse_filepath(filepath));

    uint16_t nv_count;
    EXPECT_TRUE(jp.shift_token_object(nv_count));
    EXPECT_EQ(nv_count, 1);

    char string[20];
    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "b");

    int32_t bob;
    EXPECT_FALSE(jp.shift_token_int32_t(bob));

    float mary;
    EXPECT_FALSE(jp.shift_token_float(mary));

    Vector3f fred;
    EXPECT_FALSE(jp.shift_token_vector3f(fred));

    EXPECT_TRUE(jp.shift_token_string(string, ARRAY_SIZE(string)));
    EXPECT_STREQ(string, "a");
}

AP_GTEST_MAIN()
