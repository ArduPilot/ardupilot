// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/Heap.h>
#include <AP_Module/HostApi.h>
#include <AP_Module/Image.h>
#include <AP_Module/Bindings.h>
#include <AP_Module/BindingConfig.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_MODULE_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED && HAL_OWNED_THREADS_ENABLED && defined(__linux__) && defined(__x86_64__)
#include <atomic>
#include <memory>
#include <unistd.h>
#include <vector>

class SDKModule
{
public:
    ~SDKModule()
    {
        stop.store(true);
        if (thread != nullptr && !reap()) {
            abort(); // never release an image or callback context still in use
        }
    }

    bool load(const char *path)
    {
        FILE *file = fopen(path, "rb");
        if (file == nullptr) {
            return false;
        }
        std::vector<uint8_t> bytes;
        uint8_t buffer[4096];
        size_t length;
        while ((length = fread(buffer, 1, sizeof(buffer), file)) != 0) {
            bytes.insert(bytes.end(), buffer, buffer + length);
            if (bytes.size() > 1024 * 1024) {
                fclose(file);
                return false;
            }
        }
        const bool failed = ferror(file);
        fclose(file);
        const AP_Module::Image::Compatibility expected{62, AP_MODULE_ABI_VERSION, 2, 8192, AP_MODULE_API_FINGERPRINT};
        AP_Module::Image image;
        if (failed || !image.parse(bytes.data(), bytes.size(), 1024 * 1024, expected) ||
            !image.check_imports(resolve, this, &missing_import) || !heap.create(32768)) {
            return false;
        }
        memory.reset(hal.util->allocate_executable(image.memory_size()));
        if (!memory) {
            return false;
        }
        auto *rx = static_cast<uint8_t *>(memory->data());
        const bool loaded = image.is_split() ?
                            image.load_split(rx, image.code_size(), rx + image.code_size(),
                                             image.rw_memory_size(), resolve, this) :
                            image.load(rx, memory->size(), resolve, this);
        if (!loaded || !memory->publish(image.code_size())) {
            return false;
        }
        entry = reinterpret_cast<Start>(static_cast<uint8_t *>(memory->data()) + image.entry_offset());
        return true;
    }

    bool start()
    {
        thread = hal.scheduler->thread_create_owned(FUNCTOR_BIND_MEMBER(&SDKModule::run, void),
                 "rust-sdk", 65536, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0);
        return thread != nullptr;
    }

    bool reap()
    {
        for (uint32_t i = 0; i < 10000; i++) {
            if (AP_HAL::Scheduler::thread_reap(thread)) {
                return true;
            }
            usleep(1000);
        }
        return false;
    }

    bool wait_for_sleep()
    {
        for (uint32_t i = 0; i < 10000; i++) {
            if (sleeping.load()) {
                return true;
            }
            usleep(1000);
        }
        return false;
    }

    void run()
    {
        status = entry(&api, this);
    }

    using Start = int32_t (*)(const AP_ModuleHostApi *, void *);
    AP_Module::Heap heap;
    std::unique_ptr<AP_HAL::ExecutableMemory> memory;
    AP_HAL::Scheduler::OwnedThread *thread = nullptr;
    Start entry = nullptr;
    std::atomic<bool> stop{false};
    std::atomic<bool> sleeping{false};
    bool hold_sleep = false;
    bool fail_allocation = false;
    bool callback_error = false;
    bool real_math = false;
    uint64_t mode = 0;
    uint32_t messages = 0;
    uint32_t binding_calls = 0;
    uint32_t dispatch_offset = 0;
    uint32_t removed_binding = UINT32_MAX;
    uint64_t missing_import = 0;

    static uint32_t resolve(void *context, uint64_t hash)
    {
        auto &module = *static_cast<SDKModule *>(context);
        const uint32_t id = AP_Module::binding_resolve(nullptr, hash);
        return id == UINT32_MAX || id == module.removed_binding ? UINT32_MAX : id + module.dispatch_offset;
    }
    int32_t status = -100;

    static uint8_t *allocate(void *context, uint32_t size, uint32_t alignment)
    {
        auto &module = *static_cast<SDKModule *>(context);
        return module.fail_allocation ? nullptr : static_cast<uint8_t *>(module.heap.allocate(size, alignment));
    }
    static void deallocate(void *context, uint8_t *pointer)
    {
        auto &module = *static_cast<SDKModule *>(context);
        module.callback_error |= !module.heap.deallocate(pointer);
    }
    static uint64_t time_us(void *context)
    {
        return static_cast<SDKModule *>(context)->mode;
    }
    static int32_t sleep_ms(void *context, uint32_t milliseconds)
    {
        auto &module = *static_cast<SDKModule *>(context);
        module.callback_error |= milliseconds != 60000;
        module.sleeping.store(true);
        while (module.hold_sleep && !module.stop.load()) {
            usleep(1000);
        }
        return module.stop.load() ? AP_MODULE_STATUS_STOPPED : AP_MODULE_STATUS_OK;
    }
    static uint8_t stop_requested(void *context)
    {
        return static_cast<SDKModule *>(context)->stop.load();
    }
    static int32_t text(void *context, uint8_t severity, const uint8_t *message, uint32_t length)
    {
        auto &module = *static_cast<SDKModule *>(context);
        const char *expected = module.real_math ? "MATH ok" : (module.messages == 0 ? "sdk start" : "sdk done");
        module.callback_error |= severity != 6 || length != strlen(expected) || memcmp(message, expected, length) != 0;
        module.messages++;
        return AP_MODULE_STATUS_OK;
    }
    static void panic(void *context, const uint8_t *message, uint32_t length)
    {
        // Death tests terminate their child process to observe the SDK panic
        // callback. The real runtime must park and retain the worker instead.
        fprintf(stderr, "SDK panic mode %llu: %.*s\n", (unsigned long long)static_cast<SDKModule *>(context)->mode,
                int(length), reinterpret_cast<const char *>(message));
        _exit(73);
    }
    static const AP_ModuleHostApi api;
    static int32_t binding_call(void *context, uint32_t id, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size)
    {
        auto &module = *static_cast<SDKModule *>(context);
        module.binding_calls++;
        if (id < module.dispatch_offset) {
            return AP_MODULE_STATUS_INVALID;
        }
        id -= module.dispatch_offset;
        if (module.real_math) {
            return AP_Module::binding_dispatch(nullptr, static_cast<AP_Module::BindingId>(id),
                                             input, input_size, output, output_size);
        }
        if (id == AP_MODULE_BINDING_PARAMETER_LAYOUT) {
            return AP_Module::binding_dispatch(nullptr, static_cast<AP_Module::BindingId>(id),
                                             input, input_size, output, output_size);
        }
        if (module.mode >= 8 && module.mode <= 11) {
            // SDK ownership/error tests only; the dispatcher tests exercise
            // construction and access to the real C++ Parameter separately.
            if (module.mode == 10 || (module.mode == 11 && module.binding_calls == 3)) {
                module.stop.store(true);
                return AP_MODULE_STATUS_STOPPED;
            }
            module.callback_error |= output == nullptr || output_size != 1;
            if (module.callback_error) {
                return AP_MODULE_STATUS_INVALID;
            }
            output[0] = module.mode == 8 ? 0 : (module.mode == 9 ? 2 : 1);
            return AP_MODULE_STATUS_OK;
        }
        if (module.mode == 4 || module.mode == 5) {
            module.callback_error |= output == nullptr || output_size != (module.mode == 4 ? 1U : 5U);
            if (!module.callback_error) {
                output[0] = 2; // neither a bool nor an optional presence flag
                return AP_MODULE_STATUS_OK;
            }
        }
        if (module.mode == 6) {
            return AP_MODULE_STATUS_BUSY;
        }
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
};

const AP_ModuleHostApi SDKModule::api {
    AP_MODULE_ABI_VERSION, sizeof(AP_ModuleHostApi), AP_MODULE_API_FINGERPRINT,
    allocate, deallocate, time_us, sleep_ms, stop_requested, text, panic, binding_call
};

class RustSDK : public ::testing::Test
{
protected:
    void SetUp() override
    {
        path = getenv("AP_MODULE_TEST_SDK_MODULE");
        if (path == nullptr) {
            GTEST_SKIP() << "Set AP_MODULE_TEST_SDK_MODULE to the built SDK probe";
        }
        ASSERT_TRUE(module.load(path));
    }
    const char *path = nullptr;
    SDKModule module;
};

TEST_F(RustSDK, NormalReturnAndEntryValidation)
{
    auto invalid = SDKModule::api;
    invalid.allocate = nullptr;
    EXPECT_EQ(module.entry(&invalid, &module), AP_MODULE_STATUS_INVALID);
    invalid = SDKModule::api;
    invalid.fingerprint[0] ^= 1;
    EXPECT_EQ(module.entry(&invalid, &module), AP_MODULE_STATUS_INVALID);
    EXPECT_EQ(module.entry(nullptr, &module), AP_MODULE_STATUS_INVALID);
    EXPECT_EQ(module.entry(&SDKModule::api, nullptr), AP_MODULE_STATUS_INVALID);
    alignas(AP_ModuleHostApi) const uint32_t prefix[] {AP_MODULE_ABI_VERSION, 8};
    EXPECT_EQ(module.entry(static_cast<const AP_ModuleHostApi *>(static_cast<const void *>(prefix)), &module),
              AP_MODULE_STATUS_INVALID);
    const void *misaligned = reinterpret_cast<const uint8_t *>(&invalid) + 1;
    EXPECT_EQ(module.entry(static_cast<const AP_ModuleHostApi *>(misaligned), &module), AP_MODULE_STATUS_INVALID);
    ASSERT_TRUE(module.start());
    ASSERT_TRUE(module.reap());
    EXPECT_EQ(module.status, AP_MODULE_STATUS_OK);
    EXPECT_EQ(module.messages, 2U);
    EXPECT_FALSE(module.callback_error);
    EXPECT_GT(module.heap.stats().peak, 256U);
    EXPECT_EQ(module.heap.stats().used, 0U);
    EXPECT_EQ(module.entry(&SDKModule::api, &module), AP_MODULE_STATUS_INVALID);
}

TEST_F(RustSDK, StopTwoIndependentImagesAndDropAllocations)
{
    SDKModule second;
    ASSERT_TRUE(second.load(path));
    module.hold_sleep = second.hold_sleep = true;
    ASSERT_TRUE(module.start());
    ASSERT_TRUE(second.start());
    ASSERT_TRUE(module.wait_for_sleep());
    ASSERT_TRUE(second.wait_for_sleep());
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(module.thread));
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(second.thread));
    module.stop.store(true);
    second.stop.store(true);
    ASSERT_TRUE(module.reap());
    ASSERT_TRUE(second.reap());
    for (const auto pointer : {
             &module, &second
         }) {
        EXPECT_EQ(pointer->status, AP_MODULE_STATUS_STOPPED);
        EXPECT_EQ(pointer->messages, 1U);
        EXPECT_EQ(pointer->heap.stats().used, 0U);
        EXPECT_FALSE(pointer->callback_error);
    }
}

TEST_F(RustSDK, FallibleAllocationReturnsError)
{
    module.fail_allocation = true;
    ASSERT_TRUE(module.start());
    ASSERT_TRUE(module.reap());
    EXPECT_EQ(module.status, AP_MODULE_STATUS_NO_MEMORY);
    EXPECT_EQ(module.heap.stats().used, 0U);
}

TEST_F(RustSDK, ExplicitPanicCallsHost)
{
    module.mode = 1;
    ASSERT_EXIT({ module.run(); _exit(1); }, ::testing::ExitedWithCode(73), "deliberate SDK panic");
}

TEST_F(RustSDK, RejectInvalidBindingBoolean)
{
    module.mode = 4;
    ASSERT_TRUE(module.start());
    ASSERT_TRUE(module.reap());
    EXPECT_EQ(module.status, AP_MODULE_STATUS_OK);
    EXPECT_FALSE(module.callback_error);
}

TEST_F(RustSDK, RejectInvalidBindingPresence)
{
    module.mode = 5;
    ASSERT_TRUE(module.start());
    ASSERT_TRUE(module.reap());
    EXPECT_EQ(module.status, AP_MODULE_STATUS_OK);
    EXPECT_FALSE(module.callback_error);
}

TEST_F(RustSDK, BindingContentionIsRecoverable)
{
    module.mode = 6;
    ASSERT_TRUE(module.start());
    ASSERT_TRUE(module.reap());
    EXPECT_EQ(module.status, AP_MODULE_STATUS_OK);
    EXPECT_FALSE(module.callback_error);
}

TEST_F(RustSDK, InfallibleAllocationFailureCallsHostPanic)
{
    module.mode = 2;
    module.fail_allocation = true;
    ASSERT_EXIT({ module.run(); _exit(1); }, ::testing::ExitedWithCode(73), "SDK panic mode 2:");
}

TEST_F(RustSDK, ParameterAllocationFailureAndInvalidName)
{
    for (const uint64_t mode : {
             7U, 12U
         }) {
        SDKModule probe;
        ASSERT_TRUE(probe.load(path));
        probe.mode = mode;
        probe.fail_allocation = true;
        ASSERT_TRUE(probe.start());
        ASSERT_TRUE(probe.reap());
        EXPECT_EQ(probe.status, AP_MODULE_STATUS_OK);
        EXPECT_EQ(probe.binding_calls, mode == 7 ? 1U : 0U);
        EXPECT_EQ(probe.heap.stats().used, 0U);
        EXPECT_EQ(probe.heap.stats().peak, 0U);
    }
}

TEST_F(RustSDK, ParameterDropsOnMissingNameHostErrorAndStop)
{
    for (const uint64_t mode : {
             8U, 9U, 10U, 11U
         }) {
        SDKModule probe;
        ASSERT_TRUE(probe.load(path));
        probe.mode = mode;
        ASSERT_TRUE(probe.start());
        ASSERT_TRUE(probe.reap());
        EXPECT_EQ(probe.status, AP_MODULE_STATUS_OK);
        EXPECT_EQ(probe.binding_calls, mode == 11 ? 3U : 2U);
        EXPECT_FALSE(probe.callback_error);
        EXPECT_EQ(probe.heap.stats().used, 0U);
        EXPECT_GT(probe.heap.stats().peak, 0U);
        EXPECT_EQ(probe.heap.stats().allocations, 0U);
    }
}

TEST(RustParameterABI, RejectFixedLayoutImports)
{
    // Experimental Parameter imports before the host-sized storage protocol.
    const uint64_t hashes[] {
        UINT64_C(0x823ecd93d2844080), UINT64_C(0xd83f55f38a0b265d), UINT64_C(0x355c3474cab8937a),
        UINT64_C(0xc8a7d2e7b16aa804), UINT64_C(0x4f1f4a8c3d4d4dfa), UINT64_C(0xac4b916ee8e4a87c)
    };
    for (const uint64_t hash : hashes) {
        EXPECT_EQ(AP_Module::binding_resolve(nullptr, hash), UINT32_MAX);
    }
}

TEST(RustParameterABI, RejectLegacyImage)
{
    const char *path = getenv("AP_MODULE_TEST_LEGACY_PARAMETER_MODULE");
    if (path == nullptr) {
        GTEST_SKIP() << "Set AP_MODULE_TEST_LEGACY_PARAMETER_MODULE to a pre-layout-query SDK probe";
    }
    SDKModule module;
    EXPECT_FALSE(module.load(path));
    // The SDK probe imports init/get. Require an import rejection, not an
    // unrelated file, checksum, allocation or publication failure.
    EXPECT_TRUE(module.missing_import == UINT64_C(0x355c3474cab8937a) ||
                module.missing_import == UINT64_C(0xd83f55f38a0b265d));
}

TEST_F(RustSDK, PanicDoesNotInvokeUserFormatter)
{
    module.mode = 3;
    ASSERT_EXIT({ module.run(); _exit(1); }, ::testing::ExitedWithCode(73), "formatted message omitted");
}

TEST(ModuleMathSDK, RealBindingsUseNoHeap)
{
    const char *path = getenv("AP_MODULE_TEST_MATH_MODULE");
    if (path == nullptr) {
        GTEST_SKIP() << "Set AP_MODULE_TEST_MATH_MODULE to tests/modules/math.rs built as a module";
    }
    SDKModule module;
    ASSERT_TRUE(module.load(path));
    module.real_math = true;
    module.fail_allocation = true;
    ASSERT_TRUE(module.start());
    ASSERT_TRUE(module.reap());
    EXPECT_EQ(module.status, AP_MODULE_STATUS_OK);
    EXPECT_EQ(module.messages, 1U);
    EXPECT_FALSE(module.callback_error);
    EXPECT_EQ(module.binding_calls, 3100U);
    EXPECT_EQ(module.heap.stats().used, 0U);
    EXPECT_EQ(module.heap.stats().peak, 0U);
}

static void check_compatible_image(const char *path, uint32_t expected_calls)
{
    for (const uint32_t offset : {
             0U, 1000U
         }) {
        SDKModule module;
        module.dispatch_offset = offset;
        // Simulate removal/signature change of an API this binary does not use.
        module.removed_binding = AP_MODULE_BINDING_PARAMETER_GET;
        ASSERT_TRUE(module.load(path));
        module.real_math = true;
        module.fail_allocation = true;
        ASSERT_TRUE(module.start());
        ASSERT_TRUE(module.reap());
        EXPECT_EQ(module.status, AP_MODULE_STATUS_OK);
        EXPECT_EQ(module.binding_calls, expected_calls);
        EXPECT_EQ(module.heap.stats().peak, 0U);
    }
    SDKModule incompatible;
    incompatible.removed_binding = AP_MODULE_BINDING_VECTOR3F_LENGTH;
    EXPECT_FALSE(incompatible.load(path));
    EXPECT_EQ(incompatible.binding_calls, 0U);
}

TEST(ModuleMathSDK, OldImageSurvivesPrivateRenumberingAndUnusedChanges)
{
    const char *path = getenv("AP_MODULE_TEST_MATH_MODULE");
    if (path == nullptr) {
        GTEST_SKIP() << "Set AP_MODULE_TEST_MATH_MODULE to the unchanged compiled math module";
    }
    check_compatible_image(path, 3100);
}

TEST(ModuleMathSDK, FrozenImageSurvivesPrivateRenumberingAndUnusedChanges)
{
    const char *path = getenv("AP_MODULE_TEST_COMPAT_MODULE");
    if (path == nullptr) {
        GTEST_SKIP() << "Set AP_MODULE_TEST_COMPAT_MODULE to the decoded frozen x86-64 probe";
    }
    check_compatible_image(path, 1);
}
#else
TEST(RustSDK, RequiresEnabledNativeBuild)
{
    GTEST_SKIP() << "Requires Rust, executable memory and owned threads on x86-64 Linux/SITL";
}
#endif

AP_GTEST_MAIN()
