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

class Parameters {
public:
    enum {
        k_param_old_i8,
        k_param_new_i8,
        k_param_old_v3f,
        k_param_new_v3f,
    };

    AP_Int8 old_i8;

    // new_i8 is the destination of the scalar conversion; guard follows
    // it in memory so that an over-long copy is detected
    AP_Int8 new_i8;
    uint8_t guard[CONVERSION_GUARD_LEN];

    AP_Vector3f old_v3f;
    AP_Vector3f new_v3f;
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
    // setup the var_info table
    AP_Param param_loader{var_info};
};
static TestVehicle testvehicle;

const AP_Param::Info TestVehicle::var_info[] {
    GSCALAR(old_i8,   "OLDI8",  0),
    GSCALAR(new_i8,   "NEWI8",  0),
    GSCALAR(old_v3f,  "OLDV3F", 0),
    GSCALAR(new_v3f,  "NEWV3F", 0),
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

AP_GTEST_MAIN()
