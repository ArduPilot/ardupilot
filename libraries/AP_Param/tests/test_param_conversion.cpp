#define AP_PARAM_VEHICLE_NAME testvehicle

#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// number of guard bytes placed immediately after the scalar conversion
// destination.  A copy sized by the largest storable parameter type
// (Vector3f, 12 bytes) rather than by the parameter's own type would
// write into these.
#define CONVERSION_GUARD_LEN 16

// an object used as both the source and destination of a
// convert_class() conversion.  guard follows the parameter for the same
// reason as in Parameters, below.
class ConvObj {
public:
    AP_Int8 v;
    uint8_t guard[CONVERSION_GUARD_LEN];

    static const struct AP_Param::GroupInfo var_info[];
};

const AP_Param::GroupInfo ConvObj::var_info[] = {
    AP_GROUPINFO("V", 0, ConvObj, v, 0),
    AP_GROUPEND
};

class Parameters {
public:
    enum {
        k_param_old_i8,
        k_param_new_i8,
        k_param_old_v3f,
        k_param_new_v3f,
        k_param_new_i16,
        k_param_oldobj,
        k_param_newobj,
    };

    AP_Int8 old_i8;

    // new_i8 is the destination of the scalar conversion; guard follows
    // it in memory so that an over-long copy is detected
    AP_Int8 new_i8;
    uint8_t guard[CONVERSION_GUARD_LEN];

    AP_Vector3f old_v3f;
    AP_Vector3f new_v3f;

    // destination for the parameter-width conversion tests
    AP_Int16 new_i16;
};

class TestVehicle : public AP_Vehicle {
public:
    TestVehicle() { unused_log_bitmask.set(-1); }

    // HAL::Callbacks implementation.
    void load_parameters(void) override {};
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override {
        tasks = nullptr;
        task_count = 0;
        log_bit = 0;
    };

    bool set_mode(const uint8_t new_mode, const ModeReason reason) override { return true; }
    uint8_t get_mode() const override { return 0; }

    AP_Int32 unused_log_bitmask; // logging is magic for Test; this is unused
    struct LogStructure log_structure[256] = {
    };

protected:

    const AP_Int32 &get_log_bitmask() override { return unused_log_bitmask; }
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    uint8_t get_num_log_structures() const override {
        return uint8_t(ARRAY_SIZE(log_structure));
    }

    void init_ardupilot() override {};

public:

    static const AP_Param::Info var_info[];

    Parameters g;

    // source and destination of the convert_class() conversion
    ConvObj oldobj;
    ConvObj newobj;

    // setup the var_info table
    AP_Param param_loader{var_info};
};
static TestVehicle testvehicle;

const AP_Param::Info TestVehicle::var_info[] {
    GSCALAR(old_i8,   "OLDI8",  0),
    GSCALAR(new_i8,   "NEWI8",  0),
    GSCALAR(old_v3f,  "OLDV3F", 0),
    GSCALAR(new_v3f,  "NEWV3F", 0),
    GSCALAR(new_i16,  "NEWI16", 0),
    GOBJECT(oldobj,   "OLD_",   ConvObj),
    GOBJECT(newobj,   "NEW_",   ConvObj),
    AP_VAREND
};

// start each test from an empty EEPROM so that the conversion
// destination is not already set in storage
static void reset_storage()
{
    AP_Param::erase_all();
    AP_Param::load_all();
}

// there is no IO thread here to drain the deferred save queue, so write
// the value out synchronously
static void save_now(AP_Param &p)
{
    p.save_sync(true, false);
}

// convert_old_parameter() copies the value across when the old and new
// parameters are the same type.  The copy must be the length of that
// type, not the length of the buffer the old value was read into: the
// destination parameter is only as big as its own type, so a longer copy
// walks off the end of it.
TEST(ParamConversion, ScalarConversionDoesNotOverrunDestination)
{
    reset_storage();

    // store a value for the old parameter
    testvehicle.g.old_i8.set((int8_t)0x5A);
    save_now(testvehicle.g.old_i8);

    // lay down a known pattern after the destination parameter
    memset(testvehicle.g.guard, 0xA5, sizeof(testvehicle.g.guard));

    const AP_Param::ConversionInfo info {
        Parameters::k_param_old_i8,
        0,
        AP_PARAM_INT8,
        "NEWI8"
    };
    AP_Param::convert_old_parameter(&info, 1.0f);

    // the value was converted...
    EXPECT_EQ(testvehicle.g.new_i8.get(), (int8_t)0x5A);

    // ... and nothing beyond the one-byte destination was written
    for (uint8_t i=0; i<sizeof(testvehicle.g.guard); i++) {
        EXPECT_EQ(testvehicle.g.guard[i], 0xA5)
            << "guard byte " << (int)i << " was overwritten by the conversion";
    }
}

// the counterpart to the above: the largest storable type must still be
// copied in full, so that sizing the copy by type does not truncate it
TEST(ParamConversion, VectorConversionCopiesWholeValue)
{
    reset_storage();

    const Vector3f value{1.5f, -2.25f, 3.75f};
    testvehicle.g.old_v3f.set(value);
    save_now(testvehicle.g.old_v3f);
    testvehicle.g.new_v3f.set(Vector3f{});

    const AP_Param::ConversionInfo info {
        Parameters::k_param_old_v3f,
        0,
        AP_PARAM_VECTOR3F,
        "NEWV3F"
    };
    AP_Param::convert_old_parameter(&info, 1.0f);

    EXPECT_TRUE(testvehicle.g.new_v3f.get() == value);
}

// convert_class() copies each scalar in a class across with the same
// same-type copy that convert_old_parameter() uses, and so must size
// that copy the same way
TEST(ParamConversion, ClassConversionDoesNotOverrunDestination)
{
    reset_storage();

    // store a value for the old object's parameter
    testvehicle.oldobj.v.set((int8_t)0x3C);
    save_now(testvehicle.oldobj.v);

    // lay down a known pattern after the destination parameter
    memset(testvehicle.newobj.guard, 0xA5, sizeof(testvehicle.newobj.guard));

    AP_Param::convert_class(Parameters::k_param_oldobj, &testvehicle.newobj,
                            ConvObj::var_info, 0, true);

    // the value was converted...
    EXPECT_EQ(testvehicle.newobj.v.get(), (int8_t)0x3C);

    // ... and nothing beyond the one-byte destination was written
    for (uint8_t i=0; i<sizeof(testvehicle.newobj.guard); i++) {
        EXPECT_EQ(testvehicle.newobj.guard[i], 0xA5)
            << "guard byte " << (int)i << " was overwritten by the conversion";
    }
}

// convert_parameter_width() finds the old value by scanning for a
// storage entry carrying the old type, so a successful widening can only
// be produced by storage written by an earlier firmware.  The paths
// reachable from here are the two which decline to convert.
TEST(ParamConversion, WidthConversionDeclinesWhenNothingStored)
{
    reset_storage();

    EXPECT_FALSE(testvehicle.g.new_i16.convert_parameter_width(AP_PARAM_INT8));
}

TEST(ParamConversion, WidthConversionDeclinesWhenAlreadyConfigured)
{
    reset_storage();

    // a value the user has already set must not be overwritten by a
    // conversion
    testvehicle.g.new_i16.set((int16_t)0x1234);
    save_now(testvehicle.g.new_i16);

    EXPECT_FALSE(testvehicle.g.new_i16.convert_parameter_width(AP_PARAM_INT8));
    EXPECT_EQ(testvehicle.g.new_i16.get(), (int16_t)0x1234);
}

AP_GTEST_MAIN()
