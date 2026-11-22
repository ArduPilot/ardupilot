/*
  SITL embeds Tools/autotest/pysim/vehicleinfo.json in ROMFS and, at
  startup, looks up a model's default parameter files in it (see
  SITL_State::resolve_defaults_from_romfs). Check the embedded file is
  strict JSON that AP_JSON parses, and has the shape that lookup expects
 */

#include <AP_gtest.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_JSON/AP_JSON.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// only SITL's board configuration embeds vehicleinfo.json
#if AP_FILESYSTEM_ROMFS_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL

typedef AP_JSON::value value;

static const char vehicleinfo[] = "@ROMFS/vehicleinfo.json";

// the file as it is, with no comment stripping: it must be strict JSON
TEST(AP_JSON_vehicleinfo, RawFileIsStrictJSON)
{
    FileData *fd = AP::FS().load_file(vehicleinfo);
    ASSERT_NE(fd, nullptr);
    value root;
    const std::string err = AP_JSON::parse(root, std::string((const char *)fd->data, fd->length));
    delete fd;
    EXPECT_TRUE(err.empty()) << err;
    EXPECT_TRUE(root.is<value::object>());
}

// the way SITL loads it
TEST(AP_JSON_vehicleinfo, LoadsLikeSITL)
{
    value *root = AP_JSON::load_json(vehicleinfo);
    ASSERT_NE(root, nullptr);
    ASSERT_TRUE(root->is<value::object>());
    for (const char *vehicle : { "ArduCopter", "Helicopter", "Blimp", "ArduPlane", "Rover", "ArduSub", "AntennaTracker" }) {
        EXPECT_TRUE(root->get(vehicle).is<value::object>()) << vehicle;
    }
    // a known value, to check strings come through intact
    EXPECT_EQ(std::string("default_params/copter.parm"),
              root->get("ArduCopter").get("frames").get("+").get("default_params_filename").get<std::string>());
    delete root;
}

// what resolve_frame_in_vehicle() and append_frame_defaults() rely on
TEST(AP_JSON_vehicleinfo, FramesHaveTheExpectedShape)
{
    value *root = AP_JSON::load_json(vehicleinfo);
    ASSERT_NE(root, nullptr);
    for (const auto &vehicle : root->get<value::object>()) {
        const std::string &vname = vehicle.first;
        ASSERT_TRUE(vehicle.second.is<value::object>()) << vname;
        const value &frames = vehicle.second.get("frames");
        ASSERT_TRUE(frames.is<value::object>()) << vname;
        EXPECT_FALSE(frames.get<value::object>().empty()) << vname;

        const value &default_frame = vehicle.second.get("default_frame");
        if (!default_frame.is<AP_JSON::null>()) {
            ASSERT_TRUE(default_frame.is<std::string>()) << vname;
            EXPECT_TRUE(frames.contains(default_frame.get<std::string>())) << vname;
        }

        for (const auto &frame : frames.get<value::object>()) {
            const std::string where = vname + " " + frame.first;
            ASSERT_TRUE(frame.second.is<value::object>()) << where;
            const value &model = frame.second.get("model");
            EXPECT_TRUE(model.is<AP_JSON::null>() || model.is<std::string>()) << where;
            const value &dpf = frame.second.get("default_params_filename");
            if (dpf.is<value::array>()) {
                for (const auto &name : dpf.get<value::array>()) {
                    EXPECT_TRUE(name.is<std::string>()) << where;
                }
            } else {
                EXPECT_TRUE(dpf.is<AP_JSON::null>() || dpf.is<std::string>()) << where;
            }
        }
    }
    delete root;
}

// SITL passes each default_params_filename as @ROMFS/<name>, so every one
// must be embedded too
TEST(AP_JSON_vehicleinfo, DefaultParamsAreEmbedded)
{
    value *root = AP_JSON::load_json(vehicleinfo);
    ASSERT_NE(root, nullptr);
    unsigned checked = 0;
    const auto check = [&checked](const std::string &where, const value &name) {
        if (!name.is<std::string>()) {
            return;
        }
        const std::string path = "@ROMFS/" + name.get<std::string>();
        struct stat st;
        EXPECT_EQ(0, AP::FS().stat(path.c_str(), &st)) << where << ": " << path;
        checked++;
    };
    for (const auto &vehicle : root->get<value::object>()) {
        for (const auto &frame : vehicle.second.get("frames").get<value::object>()) {
            const std::string where = vehicle.first + " " + frame.first;
            const value &dpf = frame.second.get("default_params_filename");
            if (dpf.is<value::array>()) {
                for (const auto &name : dpf.get<value::array>()) {
                    check(where, name);
                }
            } else {
                check(where, dpf);
            }
        }
    }
    EXPECT_GT(checked, 0U);
    delete root;
}

#endif  // AP_FILESYSTEM_ROMFS_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL

AP_GTEST_MAIN()
