#define AP_PARAM_VEHICLE_NAME testvehicle

#include <AP_gtest.h>
#include <AP_HAL_Empty/Scheduler.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string.h>
#include <thread>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class Parameters
{
public:
    enum { k_param_value };
    AP_Int32 value;
};

class TestVehicle : public AP_Vehicle
{
public:
    TestVehicle()
    {
        unused_log_bitmask.set(-1);
    }
    void load_parameters() override {}
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &count, uint32_t &log_bit) override
    {
        tasks = nullptr;
        count = 0;
        log_bit = 0;
    }
    bool set_mode(uint8_t, ModeReason) override
    {
        return true;
    }
    uint8_t get_mode() const override
    {
        return 0;
    }

    static const AP_Param::Info var_info[];
    Parameters g;
    AP_Param param_loader{var_info};

protected:
    void init_ardupilot() override {}
    const AP_UInt32 &get_log_bitmask() override
    {
        return unused_log_bitmask;
    }
    const LogStructure *get_log_structures() const override
    {
        return nullptr;
    }
    uint8_t get_num_log_structures() const override
    {
        return 0;
    }

private:
    AP_UInt32 unused_log_bitmask;
};
static TestVehicle testvehicle;

const AP_Param::Info TestVehicle::var_info[] = {
    GSCALAR(value, "VALUE", 0),
    AP_VAREND
};

// Deferred saves send parameter values to GCS; this test has no links.
class TestGCS : public GCS
{
public:
    uint32_t custom_mode() const override
    {
        return 0;
    }
    MAV_TYPE frame_type() const override
    {
        return MAV_TYPE_GENERIC;
    }
    GCS_MAVLINK *chan(uint8_t) override
    {
        return nullptr;
    }
    const GCS_MAVLINK *chan(uint8_t) const override
    {
        return nullptr;
    }
    GCS_MAVLINK *new_gcs_mavlink_backend(AP_HAL::UARTDriver &) override
    {
        return nullptr;
    }
};
static TestGCS test_gcs;

class FlushScheduler : public Empty::Scheduler
{
public:
    void register_io_process(AP_HAL::MemberProc proc) override
    {
        io = proc;
    }
    bool is_system_initialized() override
    {
        return false;
    }
    bool in_main_thread() const override
    {
        return std::this_thread::get_id() == main_thread;
    }
    void delay(uint16_t ms) override
    {
        delay_count++;
        delayed_ms += ms;
        if (on_delay) {
            on_delay();
        }
    }

    AP_HAL::MemberProc io;
    std::function<void()> on_delay;
    uint32_t delay_count = 0;
    uint32_t delayed_ms = 0;

private:
    const std::thread::id main_thread = std::this_thread::get_id();
};
// AP_Param registers its IO callback only once, so retain it across tests.
static FlushScheduler scheduler;

class GatedStorage : public AP_HAL::Storage
{
public:
    void init() override {}
    void read_block(void *dst, uint16_t src, size_t size) override
    {
        ASSERT_LE(size_t(src) + size, sizeof(bytes));
        memcpy(dst, &bytes[src], size);
    }
    void write_block(uint16_t dst, const void *src, size_t size) override
    {
        {
            std::unique_lock<std::mutex> lock(mutex);
            if (armed) {
                armed = false;
                entered = true;
                changed.notify_all();
                changed.wait(lock, [this] { return released; });
            }
        }
        ASSERT_LE(size_t(dst) + size, sizeof(bytes));
        memcpy(&bytes[dst], src, size);
    }
    void arm()
    {
        std::lock_guard<std::mutex> lock(mutex);
        armed = true;
    }
    bool wait_for_write()
    {
        std::unique_lock<std::mutex> lock(mutex);
        // Only a failure guard: no passing assertion depends on wall-clock timing.
        return changed.wait_for(lock, std::chrono::seconds(10), [this] { return entered; });
    }
    void release()
    {
        std::lock_guard<std::mutex> lock(mutex);
        released = true;
        changed.notify_all();
    }

private:
    uint8_t bytes[HAL_STORAGE_SIZE] {};
    std::mutex mutex;
    std::condition_variable changed;
    bool armed = false;
    bool entered = false;
    bool released = false;
};

class ParamFlush : public ::testing::Test
{
protected:
    void SetUp() override
    {
        AP_HAL::get_HAL_mutable().scheduler = &scheduler;
        AP_HAL::get_HAL_mutable().storage = &storage;
        scheduler.delay_count = 0;
        scheduler.delayed_ms = 0;
        AP_Param::erase_all();
        AP_Param::load_all();
        ASSERT_TRUE(bool(scheduler.io));
    }
    void TearDown() override
    {
        // Also release the writer after a failed assertion or an early flush return.
        finish_write();
        scheduler.on_delay = nullptr;
        AP_HAL::get_HAL_mutable().storage = old_storage;
        AP_HAL::get_HAL_mutable().scheduler = old_scheduler;
    }
    void start_write()
    {
        storage.arm();
        testvehicle.g.value.set(70000);
        testvehicle.g.value.save(true);
        worker = std::thread([] { scheduler.io(); });
    }
    void finish_write()
    {
        storage.release();
        if (worker.joinable()) {
            worker.join();
        }
    }
    void check_saved_value()
    {
        testvehicle.g.value.set(0);
        EXPECT_TRUE(testvehicle.g.value.load());
        EXPECT_EQ(testvehicle.g.value.get(), 70000);
    }

    AP_HAL::Scheduler *old_scheduler = hal.scheduler;
    AP_HAL::Storage *old_storage = hal.storage;
    GatedStorage storage;
    std::thread worker;
};

TEST_F(ParamFlush, WaitsForInFlightSave)
{
    start_write();
    ASSERT_TRUE(storage.wait_for_write());
    // The only queued save has been popped but its storage write is blocked.
    // Let it complete only when flush() waits, then join to avoid timing races.
    scheduler.on_delay = [this] { finish_write(); };
    AP_Param::flush();
    EXPECT_EQ(scheduler.delay_count, 1U);
    EXPECT_EQ(scheduler.delayed_ms, 10U);
    finish_write();
    check_saved_value();
}

TEST_F(ParamFlush, TimesOutWithInFlightSave)
{
    start_write();
    ASSERT_TRUE(storage.wait_for_write());
    // Keep the write blocked for every delay; account for simulated time only.
    AP_Param::flush();
    EXPECT_EQ(scheduler.delay_count, 200U);
    EXPECT_EQ(scheduler.delayed_ms, 2000U);
    finish_write();
    check_saved_value();
}

TEST_F(ParamFlush, EmptyQueueReturnsImmediately)
{
    AP_Param::flush();
    EXPECT_EQ(scheduler.delay_count, 0U);
    EXPECT_EQ(scheduler.delayed_ms, 0U);
}

AP_GTEST_MAIN()
