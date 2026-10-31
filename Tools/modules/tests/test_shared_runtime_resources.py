# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise native runtime ownership, startup and shared-memory constraints."""

import os
import subprocess

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def compile_run(tmp_path, source, *flags):
    path = tmp_path / 'probe.cpp'
    path.write_text(source)
    binary = tmp_path / 'probe'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(tmp_path), str(path), '-o', str(binary), *flags], check=True)
    subprocess.run([str(binary)], check=True, timeout=20)


def test_analog_none_clears_previous_owner_samples(tmp_path):
    source = (ROOT / 'libraries/AP_HAL_ChibiOS/AnalogIn.cpp').read_text()
    start = source.index('bool AnalogSource::set_pin(')
    method = source[start:source.index('\n/*', start)]
    compile_run(tmp_path, r'''
#include <cassert>
#include <cstdint>
#define ANALOG_INPUT_NONE 255
#define ANALOG_SERVO_VRSSI_PIN 254
#define ADC_GRP1_NUM_CHANNELS 1
#define WITH_SEMAPHORE(s) (void)(s)
struct AnalogIn {
    struct Pin { uint8_t analog_pin; };
    static constexpr Pin pin_config[] = {{3}};
};
struct AnalogSource {
    uint8_t _pin = 3;
    float _sum_value = 22, _sum_ratiometric = 33, _latest_value = 44;
    float _value = 55, _value_ratiometric = 66;
    uint16_t _sum_count = 4;
    int _semaphore = 0;
    bool set_pin(uint8_t pin);
};
''' + method + r'''
int main() {
    AnalogSource source;
    assert(!source.set_pin(42) && source._pin == 3 && source._value == 55);
    assert(!source.set_pin(ANALOG_INPUT_NONE));
    assert(source._pin == ANALOG_INPUT_NONE && source._sum_count == 0);
    assert(source._sum_value == 0 && source._sum_ratiometric == 0);
    assert(source._latest_value == 0 && source._value == 0 && source._value_ratiometric == 0);
    assert(source.set_pin(3) && source._value == 0);
}
''')


def test_serial_devices_need_an_enabled_runtime(tmp_path):
    source = (ROOT / 'libraries/AP_Scripting/AP_Scripting.cpp').read_text()
    start = source.index('void AP_Scripting::init_serialdevice_ports(')
    method = source[start:source.index('\n#endif', start)]
    compile_run(tmp_path, r'''
#include <cassert>
struct AP_Scripting {
    int _enable = 0;
    struct Device { int starts = 0; void init() { starts++; } } _serialdevice;
    void init_serialdevice_ports(bool native_enabled = false);
};
''' + method + r'''
int main() {
    for (int lua = 0; lua < 2; lua++) {
        for (int native = 0; native < 2; native++) {
            AP_Scripting scripting;
            scripting._enable = lua;
            scripting.init_serialdevice_ports(native);
            assert(scripting._serialdevice.starts == bool(lua || native));
        }
    }
}
''')


def test_romfs_startup_without_sd_card(tmp_path):
    source = (ROOT / 'libraries/AP_Module/Runtime.cpp').read_text()
    start = source.index('bool Runtime::load_generation()')
    discovery = source[start:source.index('    size_t count = 0;', start)] + '    return true;\n}\n'
    compile_run(tmp_path, r'''
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <dirent.h>
#define HAL_HAVE_AP_ROMFS_EMBEDDED_APMOD 1
#define WITH_SEMAPHORE(s) (void)0
static bool embedded;
static int sd_error;
namespace AP_ROMFS {
const char *dir_list(const char *, uint16_t &offset) {
    return embedded && offset++ == 0 ? "modules/embedded.apmod" : nullptr;
}
}
struct Filesystem {
    void *opendir(const char *) { errno = sd_error; return nullptr; }
    dirent *readdir(void *) { return nullptr; }
    int closedir(void *) { return 0; }
};
namespace AP { Filesystem &FS() { static Filesystem fs; return fs; } }
struct Runtime {
    enum class Phase { Loading } _phase;
    enum class ModuleSource { ROMFS, Writable };
    void *_slots = nullptr;
    char _error[64];
    int _heap_required = 0, _loaded_checksum = 0, _running_checksum = 0;
    struct Debug { int get() { return 0; } } _debug_options;
    struct Limits { bool allow_heap_expansion; } _limits;
    static constexpr const char *DIRECTORY = "./modules";
    void fail(const char *) {}
    bool add_file(const char *, ModuleSource) { _slots = this; return true; }
    bool load_generation();
};
struct Util {
    template<typename... Args> void snprintf(char *, size_t, const char *, Args...) {}
};
struct HAL { Util *util; } hal;
''' + discovery + r'''
int main() {
    for (bool present : {false, true}) {
        embedded = present;
        for (int error : {ENOENT, EIO, ENOSPC, ENODEV, EACCES}) {
            Runtime runtime;
            sd_error = error;
            const bool allowed = error == ENOENT || (present && error != EACCES);
            assert(runtime.load_generation() == allowed);
        }
    }
}
''', '-include', 'initializer_list')


def test_multiheap_expansion_preserves_shared_reserve(tmp_path):
    headers = {
        'AP_HAL/AP_HAL.h': r'''#pragma once
#include <atomic>
#include <cstring>
#include <new>
extern std::atomic<int> budget;
#define NEW_NOTHROW new (std::nothrow)
namespace AP_HAL {
struct Util {
    bool get_soft_armed() { return true; }
    unsigned available_memory() { return budget.load(); }
};
struct HAL { Util *util; };
}
''',
        'AP_HAL/Util.h': '#pragma once\n#include "AP_HAL.h"\n',
        'AP_HAL/Semaphores.h': r'''#pragma once
#include <mutex>
using HAL_Semaphore = std::mutex;
#define WITH_SEMAPHORE(s) std::lock_guard<std::mutex> lock(s)
''',
        'AP_Math/AP_Math.h': '#pragma once\n#define MIN(a,b) ((a)<(b)?(a):(b))\n'
                               '#define MAX(a,b) ((a)>(b)?(a):(b))\n',
        'AP_Scripting/AP_Scripting_config.h': '#pragma once\n#define AP_SCRIPTING_HEAP_EXPANSION_ALWAYS_ENABLED 0\n',
    }
    for name, source in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source)
    compile_run(tmp_path, r'''
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <new>
#include <thread>
void *operator new[](size_t size, const std::nothrow_t &) noexcept { return calloc(1, size); }
''' + f'#include "{ROOT}/libraries/AP_MultiHeap/AP_MultiHeap.cpp"\n' + r'''
std::atomic<int> budget{100000};
static AP_HAL::Util util;
static const AP_HAL::HAL test_hal{&util};
const AP_HAL::HAL &hal = test_hal;
struct Arena { uint32_t size; };
void *MultiHeap::heap_create(uint32_t size) {
    if (size > 1024) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
    budget -= size;
    return new Arena{size};
}
void MultiHeap::heap_destroy(void *pointer) {
    auto *arena = static_cast<Arena *>(pointer);
    budget += arena->size;
    delete arena;
}
void *MultiHeap::heap_allocate(void *pointer, uint32_t size) {
    return static_cast<Arena *>(pointer)->size < size ? nullptr : malloc(size);
}
void MultiHeap::heap_free(void *pointer) { free(pointer); }
uint32_t MultiHeap::heap_free_size(void *) const { return 0; }
int main() {
    MultiHeap heaps[2]{};
    for (auto &heap : heaps) {
        assert(heap.create(1024, 2, true, 20*1024));
        assert(heap.allocate(4096) == nullptr);
    }
    budget = 51*1024;
    std::atomic<int> ready{0};
    void *results[2]{};
    std::thread workers[2];
    for (unsigned i = 0; i < 2; i++) {
        workers[i] = std::thread([&, i]() {
            ready++;
            while (ready.load() != 2) { std::this_thread::yield(); }
            results[i] = heaps[i].allocate(4096);
        });
    }
    for (auto &worker : workers) { worker.join(); }
    assert(budget >= 20*1024);
    assert(bool(results[0]) != bool(results[1]));
    for (unsigned i = 0; i < 2; i++) {
        heaps[i].deallocate(results[i]);
        heaps[i].destroy();
    }
}
''', '-pthread')
