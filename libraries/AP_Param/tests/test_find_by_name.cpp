#define AP_PARAM_VEHICLE_NAME testvehicle

#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class Parameters {
public:
    enum {
        k_param_a,
        k_param_b,
        k_param_c,
    };
    AP_Int8 a;
    AP_Int8 b;
    AP_Int8 c;
};

class TestVehicle : public AP_Vehicle {
public:
    friend class Test;

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

    virtual bool set_mode(const uint8_t new_mode, const ModeReason reason) override { return true; }
    virtual uint8_t get_mode() const override { return 0; }

    AP_Int32 unused_log_bitmask; // logging is magic for Test; this is unused
    struct LogStructure log_structure[256] = {
    };

protected:

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
    GSCALAR(a,         "A", 0),
    GSCALAR(c,         "C", 0),
    GSCALAR(b,         "B", 0),
    GSCALAR(b,         "AA", 0),
    GSCALAR(b,         "CC", 0),
    GSCALAR(b,         "BB", 0),
};

TEST(FindByName, Bob)
{
    for (const auto &x : TestVehicle::var_info) {
        enum ap_var_type ptype = (ap_var_type)-1;
        AP_Param::ParamToken token = AP_Param::ParamToken {};
        AP_Param *p = AP_Param::find_by_name(x.name, &ptype, &token);
        EXPECT_TRUE(p);
    }
}

AP_GTEST_MAIN()
