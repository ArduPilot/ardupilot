#define AP_PARAM_VEHICLE_NAME testvehicle

#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>

#if AP_PARAM_DYNAMIC_ENABLED && defined(__linux__)
#include <atomic>
#include <thread>
#endif

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

    AP_UInt32 unused_log_bitmask; // logging is magic for Test; this is unused
    struct LogStructure log_structure[256] = {
    };

protected:

protected:

    const AP_UInt32 &get_log_bitmask() override { return unused_log_bitmask; }
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
    AP_VAREND
};

TEST(FindByName, Bob)
{
    for (const auto &x : TestVehicle::var_info) {
        if (x.type == AP_PARAM_NONE) {
            // end of the table
            break;
        }
        enum ap_var_type ptype = (ap_var_type)-1;
        AP_Param::ParamToken token = AP_Param::ParamToken {};
        AP_Param *p = AP_Param::find_by_name(x.name, &ptype, &token);
        EXPECT_TRUE(p);
    }
}

#if AP_PARAM_DYNAMIC_ENABLED
// This file uses seven of the ten shared dynamic-table slots; registration
// metadata intentionally survives each fixture's storage reset until exit.
class ParameterMemoryStorage : public AP_HAL::Storage
{
public:
    void init() override {}
    void read_block(void *destination, uint16_t source, size_t size) override
    {
        ASSERT_LE(size_t(source) + size, sizeof(bytes));
        memcpy(destination, bytes + source, size);
    }
    void write_block(uint16_t destination, const void *source, size_t size) override
    {
        ASSERT_LE(size_t(destination) + size, sizeof(bytes));
        memcpy(bytes + destination, source, size);
    }

private:
    uint8_t bytes[HAL_STORAGE_SIZE] {};
};

class PersistentTable : public ::testing::Test
{
protected:
    void SetUp() override
    {
        AP_HAL::get_HAL_mutable().storage = &storage;
        ASSERT_TRUE(AP_Param::setup());
        AP_Param::erase_all();
        ASSERT_TRUE(AP_Param::load_all());
    }
    void TearDown() override
    {
        AP_HAL::get_HAL_mutable().storage = old_storage;
    }
    AP_Float *find(const char *name)
    {
        ap_var_type type;
        AP_Param *parameter = AP_Param::find(name, &type);
        return parameter != nullptr && type == AP_PARAM_FLOAT ? static_cast<AP_Float *>(parameter) : nullptr;
    }

    AP_HAL::Storage *old_storage = hal.storage;
    ParameterMemoryStorage storage;
};

#if defined(__linux__)
// Keep this before other dynamic-table tests to exercise lazy allocation as
// well as concurrent claims of one slot and first publication of its names.
// A functional pass alone need not detect a race. Check writer serialization:
// Run with --gtest_filter='PersistentTable.Concurrent*' under:
// valgrind --tool=helgrind --error-exitcode=99 build/linux/tests/test_find_by_name
// This does not test the unchanged lock-free readers against registration.
TEST_F(PersistentTable, ConcurrentFirstRegistration)
{
    const uint16_t before = AP_Param::count_parameters();
    const bool existing = find("RTHR_ONE") != nullptr;
    std::atomic<unsigned> ready{0};
    std::atomic<bool> start{false};
    std::atomic<bool> failed{false};
    std::thread workers[8];
    for (auto &worker : workers) {
        const bool second = (&worker - workers) % 2 != 0;
        worker = std::thread([&, second]() {
            ready.fetch_add(1);
            while (!start.load()) {
                std::this_thread::yield();
            }
            for (unsigned iteration = 0; iteration < 100; iteration++) {
                const uint8_t key = second ? 76 : 75;
                if (!AP_Param::add_table_persistent(key, second ? "RTHS_" : "RTHR_", 2) ||
                    !AP_Param::add_param(key, 1, "ONE", second ? 30.0f : 10.0f) ||
                    !AP_Param::add_param(key, 2, "TWO", second ? 40.0f : 20.0f)) {
                    failed.store(true);
                }
            }
        });
    }
    while (ready.load() != ARRAY_SIZE(workers)) {
        std::this_thread::yield();
    }
    start.store(true);
    for (auto &worker : workers) {
        worker.join();
    }
    ASSERT_FALSE(failed.load());
    AP_Float *one = find("RTHR_ONE");
    AP_Float *two = find("RTHR_TWO");
    ASSERT_NE(one, nullptr);
    ASSERT_NE(two, nullptr);
    EXPECT_NE(one, two);
    EXPECT_FLOAT_EQ(one->get(), 10.0f);
    EXPECT_FLOAT_EQ(two->get(), 20.0f);
    AP_Float *other_one = find("RTHS_ONE");
    AP_Float *other_two = find("RTHS_TWO");
    ASSERT_NE(other_one, nullptr);
    ASSERT_NE(other_two, nullptr);
    EXPECT_NE(other_one, one);
    EXPECT_NE(other_two, two);
    EXPECT_FLOAT_EQ(other_one->get(), 30.0f);
    EXPECT_FLOAT_EQ(other_two->get(), 40.0f);
    EXPECT_EQ(AP_Param::count_parameters(), before + (existing ? 0 : 4));
    EXPECT_FALSE(AP_Param::add_table(75, "RTHR_", 2));
    EXPECT_FALSE(AP_Param::add_table_persistent(75, "OTHER_", 2));
}

TEST_F(PersistentTable, ConcurrentRegistrationModeConflict)
{
    std::atomic<unsigned> ready{0};
    std::atomic<bool> start{false};
    bool legacy = false;
    bool persistent = false;
    auto register_table = [&](bool keep, bool &result) {
        ready.fetch_add(1);
        while (!start.load()) {
            std::this_thread::yield();
        }
        result = keep ? AP_Param::add_table_persistent(77, "RRACE_", 1) : AP_Param::add_table(77, "RRACE_", 1);
    };
    std::thread first([&]() { register_table(false, legacy); });
    std::thread second([&]() { register_table(true, persistent); });
    while (ready.load() != 2) {
        std::this_thread::yield();
    }
    start.store(true);
    first.join();
    second.join();
    EXPECT_NE(legacy, persistent); // exactly one ownership mode wins
}
#endif

TEST_F(PersistentTable, ReloadRestoresSavedValue)
{
    ASSERT_TRUE(AP_Param::add_table_persistent(70, "RLOAD_", 3));
    ASSERT_TRUE(AP_Param::add_param(70, 1, "GAIN", 1.0f));
    ASSERT_TRUE(AP_Param::add_param(70, 3, "OTHER", 2.0f));
    AP_Float *gain = find("RLOAD_GAIN");
    ASSERT_NE(gain, nullptr);
    gain->set(42.0f);
    // No IO thread in this test; commit the value synchronously to the fake storage.
    gain->save_sync(true, false);
    ASSERT_TRUE(gain->load());
    const uint16_t count = AP_Param::count_parameters();
    for (uint16_t i = 0; i < 1000; i++) {
        gain->set(99.0f); // unsaved change must not override the saved value
        ASSERT_TRUE(AP_Param::add_table_persistent(70, "RLOAD_", 3));
        ASSERT_EQ(find("RLOAD_GAIN"), gain);
        ASSERT_NE(find("RLOAD_OTHER"), nullptr);
        ASSERT_EQ(AP_Param::count_parameters(), count);
        ASSERT_TRUE(AP_Param::add_param(70, 1, "GAIN", 3.0f));
        ASSERT_FLOAT_EQ(gain->get(), 42.0f);
    }
}

TEST_F(PersistentTable, RejectsIdentityChanges)
{
    ASSERT_TRUE(AP_Param::add_table_persistent(71, "RFIX_", 2));
    ASSERT_TRUE(AP_Param::add_param(71, 1, "VALUE", 5.0f));
    EXPECT_FALSE(AP_Param::add_table_persistent(71, "CHANGED_", 2));
    EXPECT_FALSE(AP_Param::add_table_persistent(71, "RFIX_", 3));
    EXPECT_FALSE(AP_Param::add_param(71, 1, "RENAMED", 6.0f));
    EXPECT_FALSE(AP_Param::add_param(71, 2, "VALUE", 6.0f));
    EXPECT_FALSE(AP_Param::add_param(71, 3, "EXTRA", 6.0f));
    EXPECT_FALSE(AP_Param::add_table(71, "RFIX_", 2));
    EXPECT_TRUE(AP_Param::add_table_persistent(71, "RFIX_", 1));
    // Smaller declarations do not truncate firmware-owned metadata or storage.
    EXPECT_TRUE(AP_Param::add_param(71, 2, "SECOND", 7.0f));
    ASSERT_TRUE(AP_Param::add_table(72, "LEGACY_", 1));
    EXPECT_FALSE(AP_Param::add_table_persistent(72, "LEGACY_", 1));
    EXPECT_FALSE(AP_Param::add_table_persistent(73, nullptr, 1));
    EXPECT_FALSE(AP_Param::add_table_persistent(73, "EMPTY_", 0));
    EXPECT_FALSE(AP_Param::add_table_persistent(73, "BIG_", 64));
}

TEST_F(PersistentTable, UnsavedValueReloadsDefault)
{
    ASSERT_TRUE(AP_Param::add_table_persistent(74, "RDEF_", 1));
    ASSERT_TRUE(AP_Param::add_param(74, 1, "VALUE", 5.0f));
    AP_Float *value = find("RDEF_VALUE");
    ASSERT_NE(value, nullptr);
    value->set(20.0f);
    ASSERT_TRUE(AP_Param::add_table_persistent(74, "RDEF_", 1));
    ASSERT_TRUE(AP_Param::add_param(74, 1, "VALUE", 7.0f));
    EXPECT_FLOAT_EQ(value->get(), 7.0f);
}
#if AP_PARAM_DEFAULTS_ENABLED && defined(__linux__)
class DefaultAccess : public AP_Param
{
public:
    using AP_Param::add_default;
};

TEST_F(PersistentTable, ConcurrentDefaults)
{
    AP_Param *const parameters[] = {&testvehicle.g.a, &testvehicle.g.b, &testvehicle.g.c};
    std::atomic<unsigned> ready{0};
    std::atomic<bool> start{false};
    std::atomic<bool> failed{false};
    std::thread writers[6];
    for (unsigned index = 0; index < ARRAY_SIZE(writers); index++) {
        writers[index] = std::thread([&, index]() {
            ready.fetch_add(1);
            while (!start.load()) {
                std::this_thread::yield();
            }
            for (unsigned iteration = 0; iteration < 200; iteration++) {
                // Exercise list insertion/update without racing on the scalar
                // parameter value, which is deliberately outside this lock.
                DefaultAccess::add_default(parameters[index % ARRAY_SIZE(parameters)], 10.0f + index);
            }
        });
    }
    std::thread reader([&]() {
        ready.fetch_add(1);
        while (!start.load()) {
            std::this_thread::yield();
        }
        // Bound the work rather than spinning on completion while taking the
        // same mutex as writers; that can starve them under race detectors.
        for (unsigned iteration = 0; iteration < 200; iteration++) {
            AP_Param::ParamToken token{};
            ap_var_type type;
            float value;
            for (AP_Param *p = AP_Param::first(&token, &type, &value); p != nullptr;
                 p = AP_Param::next_scalar(&token, &type, &value)) {
                if (!std::isfinite(value)) {
                    failed.store(true);
                }
            }
        }
    });
    while (ready.load() != ARRAY_SIZE(writers) + 1) {
        std::this_thread::yield();
    }
    start.store(true);
    for (auto &writer : writers) {
        writer.join();
    }
    reader.join();
    EXPECT_FALSE(failed.load());
    for (unsigned index = 0; index < ARRAY_SIZE(parameters); index++) {
        DefaultAccess::add_default(parameters[index], 100.0f + index);
    }
    unsigned seen = 0;
    AP_Param::ParamToken token{};
    ap_var_type type;
    float value;
    for (AP_Param *p = AP_Param::first(&token, &type, &value); p != nullptr;
         p = AP_Param::next_scalar(&token, &type, &value)) {
        for (unsigned index = 0; index < ARRAY_SIZE(parameters); index++) {
            if (p == parameters[index]) {
                EXPECT_FLOAT_EQ(value, 100.0f + index);
                seen |= 1U << index;
            }
        }
    }
    EXPECT_EQ(seen, 7U);
}
#endif
#endif // AP_PARAM_DYNAMIC_ENABLED

AP_GTEST_MAIN()
