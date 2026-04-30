#include <AP_gtest.h>

#include <SITL/SIM_JSON.h>
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace SITL;

// Friend class declared in SIM_JSON.h that grants access to private members.
class JSONTest {
public:
    static uint64_t parse_sensors(SITL::JSON &j, const char *json)
    {
        return j.parse_sensors(json);
    }

    // --- state accessors ---
    static double get_latitude(SITL::JSON &j)    { return j.state.latitude; }
    static double get_longitude(SITL::JSON &j)   { return j.state.longitude; }
    static double get_altitude(SITL::JSON &j)    { return j.state.altitude; }
    static float  get_bat_volt(SITL::JSON &j)    { return j.state.bat_volt; }
    static float  get_bat_amp(SITL::JSON &j)     { return j.state.bat_amp; }
    static float  get_rng(SITL::JSON &j, uint8_t i)   { return j.state.rng[i]; }
    static float  get_rc(SITL::JSON &j, uint8_t i)    { return j.state.rc[i]; }
    static float  get_airspeed(SITL::JSON &j)    { return j.state.airspeed; }
    static float  get_rpm(SITL::JSON &j, uint8_t i)   { return j.state.rpm[i]; }
    static bool   get_no_lockstep(SITL::JSON &j) { return j.state.no_lockstep; }
    static float  get_wind_dir(SITL::JSON &j)    { return j.state.wind_vane_apparent.direction; }
    static float  get_wind_spd(SITL::JSON &j)    { return j.state.wind_vane_apparent.speed; }
    static Quaternion get_quaternion(SITL::JSON &j) { return j.state.quaternion; }

    // --- named bitmask accessors (private enum visible through friendship) ---
    static uint64_t TIMESTAMP_BIT() { return (uint64_t)SITL::JSON::TIMESTAMP; }
    static uint64_t EULER_ATT_BIT() { return (uint64_t)SITL::JSON::EULER_ATT; }
    static uint64_t QUAT_ATT_BIT()  { return (uint64_t)SITL::JSON::QUAT_ATT;  }
    static uint64_t BAT_VOLT_BIT()  { return (uint64_t)SITL::JSON::BAT_VOLT;  }
    static uint64_t BAT_AMP_BIT()   { return (uint64_t)SITL::JSON::BAT_AMP;   }
    static uint64_t AIRSPEED_BIT()  { return (uint64_t)SITL::JSON::AIRSPEED;  }
    static uint64_t WIND_DIR_BIT()  { return (uint64_t)SITL::JSON::WIND_DIR;  }
    static uint64_t WIND_SPD_BIT()  { return (uint64_t)SITL::JSON::WIND_SPD;  }
    static uint64_t LOCKSTEP_BIT()  { return (uint64_t)SITL::JSON::LOCKSTEP;  }
    static uint64_t RNG_BIT(uint8_t i) { return (uint64_t)SITL::JSON::RNG_1 << i; }
    static uint64_t RC_BIT(uint8_t i)  { return (uint64_t)SITL::JSON::RC_1  << i; }
    static uint64_t RPM_BIT(uint8_t i) { return (uint64_t)SITL::JSON::RPM_1 << i; }
    static uint64_t RPM_ALL() {
        return (uint64_t)SITL::JSON::RPM_1 | (uint64_t)SITL::JSON::RPM_2 |
               (uint64_t)SITL::JSON::RPM_3 | (uint64_t)SITL::JSON::RPM_4;
    }
};

// Fields parse_sensors requires (required=true in keytable).
// Attitude is optional but included to make the packet realistic.
#define MANDATORY \
    "\"timestamp\": 1.0," \
    "\"imu\": {\"gyro\": [0,0,0], \"accel_body\": [0,0,9.8]}," \
    "\"velocity\": [0,0,0]," \
    "\"attitude\": [0,0,0],"

// ---------------------------------------------------------------------------
// Mandatory field validation
// ---------------------------------------------------------------------------

// Each entry omits one required field; parse_sensors must return 0.
const char *missing_mandatory_cases[] = {
    // missing timestamp
    "{\"imu\": {\"gyro\": [0,0,0], \"accel_body\": [0,0,9.8]}, \"velocity\": [0,0,0], \"attitude\": [0,0,0]}",
    // missing gyro
    "{\"timestamp\": 1.0, \"imu\": {\"accel_body\": [0,0,9.8]}, \"velocity\": [0,0,0], \"attitude\": [0,0,0]}",
    // missing accel_body
    "{\"timestamp\": 1.0, \"imu\": {\"gyro\": [0,0,0]}, \"velocity\": [0,0,0], \"attitude\": [0,0,0]}",
    // missing velocity
    "{\"timestamp\": 1.0, \"imu\": {\"gyro\": [0,0,0], \"accel_body\": [0,0,9.8]}, \"attitude\": [0,0,0]}",
};

TEST(JSON, missing_mandatory_field_returns_zero)
{
    for (auto json : missing_mandatory_cases) {
        SITL::JSON j("json");
        EXPECT_EQ(JSONTest::parse_sensors(j, json), 0ULL) << "Expected 0 for: " << json;
    }
}

TEST(JSON, mandatory_fields_set_correct_bits)
{
    SITL::JSON j("json");
    const char *json = "{" MANDATORY "}";
    uint64_t mask = JSONTest::parse_sensors(j, json);

    EXPECT_NE(mask, 0ULL);
    EXPECT_TRUE(mask & JSONTest::TIMESTAMP_BIT());
    EXPECT_TRUE(mask & JSONTest::EULER_ATT_BIT());
    // With only attitude present (not quaternion), QUAT_ATT must be clear.
    EXPECT_EQ(mask & JSONTest::QUAT_ATT_BIT(), 0ULL);
    // No optional sections sent — RPM bits must all be clear.
    EXPECT_EQ(mask & JSONTest::RPM_ALL(), 0ULL);
}

// ---------------------------------------------------------------------------
// Attitude: euler vs quaternion are mutually exclusive in the bitmask.
// ---------------------------------------------------------------------------

TEST(JSON, quaternion_sets_quat_bit_not_euler_bit)
{
    SITL::JSON j("json");
    // Send quaternion instead of attitude; omit the "attitude" key
    const char *json =
        "{\"timestamp\": 1.0,"
        "\"imu\": {\"gyro\": [0,0,0], \"accel_body\": [0,0,9.8]},"
        "\"velocity\": [0,0,0],"
        "\"quaternion\": [1.0, 0.0, 0.0, 0.0]}";

    uint64_t mask = JSONTest::parse_sensors(j, json);
    EXPECT_NE(mask, 0ULL);
    EXPECT_TRUE(mask  & JSONTest::QUAT_ATT_BIT())  << "QUAT_ATT should be set";
    EXPECT_EQ(mask & JSONTest::EULER_ATT_BIT(), 0ULL) << "EULER_ATT should not be set";

    Quaternion q = JSONTest::get_quaternion(j);
    EXPECT_NEAR(q.q1, 1.0f, 0.001f);
    EXPECT_NEAR(q.q2, 0.0f, 0.001f);
    EXPECT_NEAR(q.q3, 0.0f, 0.001f);
    EXPECT_NEAR(q.q4, 0.0f, 0.001f);
}

// ---------------------------------------------------------------------------
// GPS precision — stored as double; a float cast would silently lose ~1m.
// ---------------------------------------------------------------------------

const struct {
    double lat, lng, alt;
} gps_table[] = {
    {  51.4778,    -0.0014,   11.0 },    // Greenwich
    { -33.8688,   151.2093,   50.0 },    // Sydney
    {  90.0,        0.0,    8848.0 },    // extreme: pole + high altitude
    {   0.0,      180.0,      -0.5 },    // equator, dateline, below sea level
    {  37.123456789, -122.123456789, 123.456 }, // high-precision: needs double
};

TEST(JSON, gps_field_precision)
{
    for (auto &elem : gps_table) {
        SITL::JSON j("json");
        char json[512];
        snprintf(json, sizeof(json),
            "{" MANDATORY
            "\"latitude\": %.9f, \"longitude\": %.9f, \"altitude\": %.3f}",
            elem.lat, elem.lng, elem.alt);

        JSONTest::parse_sensors(j, json);
        EXPECT_NEAR(JSONTest::get_latitude(j),  elem.lat, 1e-7) << "latitude";
        EXPECT_NEAR(JSONTest::get_longitude(j), elem.lng, 1e-7) << "longitude";
        EXPECT_NEAR(JSONTest::get_altitude(j),  elem.alt, 0.001) << "altitude";
    }
}

// ---------------------------------------------------------------------------
// Battery — voltage and current must be stored independently.
// ---------------------------------------------------------------------------

const struct {
    float volt, amp;
} battery_table[] = {
    { 12.6f,  5.0f   },
    {  0.0f,  0.0f   },   // zero values: bits must still be set
    { 16.8f, 80.0f   },   // 4S LiPo at high current
    {  3.0f,  0.001f },   // low voltage, near-zero current
};

TEST(JSON, battery_voltage_and_current)
{
    for (auto &elem : battery_table) {
        SITL::JSON j("json");
        char json[256];
        snprintf(json, sizeof(json),
            "{" MANDATORY
            "\"battery\": {\"voltage\": %f, \"current\": %f}}",
            elem.volt, elem.amp);

        uint64_t mask = JSONTest::parse_sensors(j, json);
        EXPECT_TRUE(mask & JSONTest::BAT_VOLT_BIT()) << "BAT_VOLT bit missing";
        EXPECT_TRUE(mask & JSONTest::BAT_AMP_BIT())  << "BAT_AMP bit missing";
        EXPECT_NEAR(JSONTest::get_bat_volt(j), elem.volt, 0.001f) << "voltage";
        EXPECT_NEAR(JSONTest::get_bat_amp(j),  elem.amp,  0.001f) << "current";
    }
}

// ---------------------------------------------------------------------------
// RC channels — spot-check non-adjacent channels to catch index mapping bugs.
// ---------------------------------------------------------------------------

const struct {
    uint8_t chan;   // 0-based index into keytable / state.rc[]
    float   value;
} rc_table[] = {
    {  0, 1000.0f },   // rc_1
    {  3, 1500.0f },   // rc_4
    {  6, 1800.0f },   // rc_7
    { 11, 2000.0f },   // rc_12
};

TEST(JSON, rc_channel_values)
{
    for (auto &elem : rc_table) {
        SITL::JSON j("json");
        char json[256];
        snprintf(json, sizeof(json),
            "{" MANDATORY "\"rc\": {\"rc_%u\": %f}}",
            elem.chan + 1, elem.value);

        uint64_t mask = JSONTest::parse_sensors(j, json);
        EXPECT_TRUE(mask & JSONTest::RC_BIT(elem.chan))
            << "RC bit " << (int)elem.chan << " not set";
        // The adjacent lower channel must not be reported.
        if (elem.chan > 0) {
            EXPECT_EQ(mask & JSONTest::RC_BIT(elem.chan - 1), 0ULL)
                << "adjacent lower RC bit should be clear for chan " << (int)elem.chan;
        }
        EXPECT_NEAR(JSONTest::get_rc(j, elem.chan), elem.value, 0.01f)
            << "rc[" << (int)elem.chan << "]";
    }
}

// ---------------------------------------------------------------------------
// Rangefinders — six independent channels; test first, middle, and last.
// ---------------------------------------------------------------------------

const struct {
    uint8_t idx;   // 0-based
    float   dist;
} rng_table[] = {
    { 0, 1.5f  },
    { 2, 10.0f },
    { 5, 0.25f },
};

TEST(JSON, rangefinder_values)
{
    for (auto &elem : rng_table) {
        SITL::JSON j("json");
        char json[256];
        snprintf(json, sizeof(json),
            "{" MANDATORY "\"rng_%u\": %f}",
            elem.idx + 1, elem.dist);

        uint64_t mask = JSONTest::parse_sensors(j, json);
        EXPECT_TRUE(mask & JSONTest::RNG_BIT(elem.idx))
            << "RNG bit " << (int)elem.idx << " not set";
        EXPECT_NEAR(JSONTest::get_rng(j, elem.idx), elem.dist, 0.001f)
            << "rng[" << (int)elem.idx << "]";
    }
}

// ---------------------------------------------------------------------------
// Airspeed
// ---------------------------------------------------------------------------

TEST(JSON, airspeed_parsed)
{
    SITL::JSON j("json");
    const char *json = "{" MANDATORY "\"airspeed\": 25.4}";
    uint64_t mask = JSONTest::parse_sensors(j, json);
    EXPECT_TRUE(mask & JSONTest::AIRSPEED_BIT());
    EXPECT_NEAR(JSONTest::get_airspeed(j), 25.4f, 0.01f);
}

// ---------------------------------------------------------------------------
// Wind vane — direction and speed stored independently.
// ---------------------------------------------------------------------------

TEST(JSON, wind_vane_direction_and_speed)
{
    SITL::JSON j("json");
    const char *json =
        "{" MANDATORY
        "\"windvane\": {\"direction\": 1.57, \"speed\": 8.5}}";

    uint64_t mask = JSONTest::parse_sensors(j, json);
    EXPECT_TRUE(mask & JSONTest::WIND_DIR_BIT());
    EXPECT_TRUE(mask & JSONTest::WIND_SPD_BIT());
    EXPECT_NEAR(JSONTest::get_wind_dir(j), 1.57f, 0.001f);
    EXPECT_NEAR(JSONTest::get_wind_spd(j), 8.5f,  0.001f);
}

// ---------------------------------------------------------------------------
// Boolean parsing — no_lockstep must accept "true", "false", "1", "0".
// ---------------------------------------------------------------------------

const struct {
    const char *token;
    bool expected;
} bool_table[] = {
    { "true",  true  },
    { "false", false },
    { "1",     true  },
    { "0",     false },
};

TEST(JSON, boolean_no_lockstep_variants)
{
    for (auto &elem : bool_table) {
        SITL::JSON j("json");
        char json[256];
        snprintf(json, sizeof(json),
            "{" MANDATORY "\"no_lockstep\": %s}", elem.token);

        uint64_t mask = JSONTest::parse_sensors(j, json);
        EXPECT_TRUE(mask & JSONTest::LOCKSTEP_BIT())
            << "LOCKSTEP bit not set for token: " << elem.token;
        EXPECT_EQ(JSONTest::get_no_lockstep(j), elem.expected)
            << "no_lockstep mismatch for token: " << elem.token;
    }
}

// ---------------------------------------------------------------------------
// RPM parsing
// ---------------------------------------------------------------------------

const struct {
    float rpm[4];
} rpm_table[] = {
    { { 1000.0f, 2000.0f, 3000.0f, 4000.0f } },
    { {    0.0f,    0.0f,    0.0f,    0.0f } },  // zero: bit must still be set
    { { 9999.9f, 8888.8f, 7777.7f, 6666.6f } },
    { {   -1.0f,   -1.0f,   -1.0f,   -1.0f } },  // negative (e.g. reverse spin)
};

TEST(JSON, rpm_all_four_motors)
{
    for (auto &elem : rpm_table) {
        SITL::JSON j("json");
        char json[256];
        snprintf(json, sizeof(json),
            "{" MANDATORY
            "\"rpm\": {\"rpm_1\": %f, \"rpm_2\": %f, \"rpm_3\": %f, \"rpm_4\": %f}}",
            elem.rpm[0], elem.rpm[1], elem.rpm[2], elem.rpm[3]);

        uint64_t mask = JSONTest::parse_sensors(j, json);
        for (uint8_t i = 0; i < 4; i++) {
            EXPECT_TRUE(mask & JSONTest::RPM_BIT(i))
                << "RPM bit " << (int)i << " not set";
            EXPECT_NEAR(JSONTest::get_rpm(j, i), elem.rpm[i], 0.01f)
                << "rpm[" << (int)i << "]";
        }
    }
}

// Only rpm_1 and rpm_3 present — exactly those two bits must be set.
TEST(JSON, rpm_partial_motors_only_correct_bits_set)
{
    SITL::JSON j("json");
    const char *json =
        "{" MANDATORY
        "\"rpm\": {\"rpm_1\": 500.0, \"rpm_3\": 1500.0}}";

    uint64_t mask = JSONTest::parse_sensors(j, json);
    EXPECT_TRUE(mask  & JSONTest::RPM_BIT(0)) << "RPM_1 should be set";
    EXPECT_EQ(mask & JSONTest::RPM_BIT(1), 0ULL) << "RPM_2 should NOT be set";
    EXPECT_TRUE(mask  & JSONTest::RPM_BIT(2)) << "RPM_3 should be set";
    EXPECT_EQ(mask & JSONTest::RPM_BIT(3), 0ULL) << "RPM_4 should NOT be set";
    EXPECT_NEAR(JSONTest::get_rpm(j, 0), 500.0f,  0.01f);
    EXPECT_NEAR(JSONTest::get_rpm(j, 2), 1500.0f, 0.01f);
}

// ---------------------------------------------------------------------------
// Cross-section isolation — battery and rpm sections must not contaminate
// each other's bits, including when both are present in the same packet.
// ---------------------------------------------------------------------------

TEST(JSON, battery_and_rpm_do_not_contaminate_each_other)
{
    // Battery only — RPM bits must all be clear.
    {
        SITL::JSON j("json");
        const char *json = "{" MANDATORY "\"battery\": {\"voltage\": 4.2, \"current\": 1.0}}";
        uint64_t mask = JSONTest::parse_sensors(j, json);
        EXPECT_TRUE(mask & JSONTest::BAT_VOLT_BIT());
        EXPECT_EQ(mask & JSONTest::RPM_ALL(), 0ULL) << "RPM bits set by battery-only packet";
    }
    // RPM only — battery bits must be clear.
    {
        SITL::JSON j("json");
        const char *json = "{" MANDATORY "\"rpm\": {\"rpm_1\": 3000.0}}";
        uint64_t mask = JSONTest::parse_sensors(j, json);
        EXPECT_TRUE(mask & JSONTest::RPM_BIT(0));
        EXPECT_EQ(mask & JSONTest::BAT_VOLT_BIT(), 0ULL) << "BAT_VOLT set by rpm-only packet";
        EXPECT_EQ(mask & JSONTest::BAT_AMP_BIT(),  0ULL) << "BAT_AMP set by rpm-only packet";
    }
    // Both present — all expected bits set, no spill-over.
    {
        SITL::JSON j("json");
        const char *json =
            "{" MANDATORY
            "\"battery\": {\"voltage\": 12.6, \"current\": 5.0},"
            "\"rpm\": {\"rpm_1\": 1000.0, \"rpm_2\": 2000.0, \"rpm_3\": 3000.0, \"rpm_4\": 4000.0}}";
        uint64_t mask = JSONTest::parse_sensors(j, json);
        EXPECT_TRUE(mask & JSONTest::BAT_VOLT_BIT());
        EXPECT_TRUE(mask & JSONTest::BAT_AMP_BIT());
        for (uint8_t i = 0; i < 4; i++) {
            EXPECT_TRUE(mask & JSONTest::RPM_BIT(i)) << "RPM bit " << (int)i;
        }
        EXPECT_NEAR(JSONTest::get_bat_volt(j), 12.6f, 0.001f);
        EXPECT_NEAR(JSONTest::get_rpm(j, 1), 2000.0f, 0.01f);
    }
}

// ---------------------------------------------------------------------------
// RPM bitmask positions — verify the enum sits at the expected bit indices
// (36–39).  A keytable entry inserted in the wrong position would shift these.
// ---------------------------------------------------------------------------

TEST(JSON, rpm_enum_bit_positions)
{
    EXPECT_EQ(JSONTest::RPM_BIT(0), 1ULL << 36);
    EXPECT_EQ(JSONTest::RPM_BIT(1), 1ULL << 37);
    EXPECT_EQ(JSONTest::RPM_BIT(2), 1ULL << 38);
    EXPECT_EQ(JSONTest::RPM_BIT(3), 1ULL << 39);
}

AP_GTEST_MAIN()
