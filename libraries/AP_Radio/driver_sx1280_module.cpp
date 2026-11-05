// SPDX-License-Identifier: GPL-3.0-or-later
#include "driver_sx1280_module.h"
#if defined(AP_MODULE_SX1280_ENABLED) && AP_MODULE_SX1280_ENABLED
#include <AP_HAL/utility/VirtualUARTDriver.h>
#include <atomic>
#include <AP_Math/AP_Math.h>
#include <string.h>
extern const AP_HAL::HAL &hal;
namespace SX1280Module
{
static HAL_Semaphore gate;
static void *owner;
static void *config_owner;
static Entry entry;
static void *rw;
alignas(8) static uint8_t state[4096];
static uint8_t settings[12];
static uint8_t latest[256];
static bool prepared, started;
static std::atomic<uint32_t> epoch, calls, rejected, unavailable, faults, maximum_us;
static uint8_t down[62], down_size, up_ack;
static uint32_t main_epoch, main_generation;
static bool drained;
static uint8_t request[128], response[256];
static bool request_pending, response_ready;
static uint32_t get32(const uint8_t *p)
{
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
static void put32(uint8_t *p, uint32_t n)
{
    for (uint8_t i = 0; i < 4; i++) {
        p[i] = n >> (8 * i);
    }
}
#if defined(__ARM_ARCH_7EM__)
__attribute__((naked, noinline)) static uint32_t invoke(Entry fn, uint8_t *s, const uint8_t *in, uint8_t *out, void *base)
{
    asm volatile("push {r9, lr}\nmov r12, r0\nmov r0, r1\nmov r1, r2\nmov r2, r3\nldr r9, [sp, #8]\nblx r12\npop {r9, pc}\n");
}
#else
static uint32_t invoke(Entry fn, uint8_t *s, const uint8_t *in, uint8_t *out, void *)
{
    return fn(s, in, out);
}
#endif
static bool call(const uint8_t in[128], uint8_t out[256])
{
    const uint32_t start = AP_HAL::micros();
    const uint32_t result = invoke(entry, state, in, out, rw);
    const uint32_t elapsed = AP_HAL::micros() - start;
    calls++;
    if (elapsed > maximum_us.load()) {
        maximum_us = elapsed;
    }
    if (result != 1 || out[0] != 1) {
        rejected++;
        return false;
    }
    if ((out[1] & ~31U) != 0 || out[25] > 16 || out[43] > 62 || out[112] > 16 || get32(out + 4) > 1000000) {
        faults++; return false;
    }
    return true;
}
class Device
{
public:
    AP_SX1280 radio;
    void worker();
    bool start()
    {
        return hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&Device::worker, void), "RFModule", 2048,
                                            AP_HAL::Scheduler::PRIORITY_SPI, 1);
    }
    void packet(const uint8_t *data);
private:
    uint8_t input[128] {};
    uint8_t size = 0;
    bool pending = false;
    uint32_t generation = 0, frequency = 0;
};
static Device device;

bool prepare(void *requester, const uint8_t config[12])
{
    WITH_SEMAPHORE(gate);
    if (!requester || owner || (config_owner && config_owner != requester)) {
        return false;
    }
    config_owner = requester;
    memcpy(settings, config, 12);
    if (!prepared) {
        if (auto *uart = AP_HAL::VirtualUARTDriver::get_uart(AP_RADIO_SX1280_UART)) {
            uart->begin(0, 2048, 4096);
        }
    }
    prepared = true;
    return true;
}
bool install(void *new_owner, Entry fn, void *base)
{
    WITH_SEMAPHORE(gate);
    if (!prepared || owner || !new_owner || config_owner != new_owner || !fn) {
        return false;
    }
    uint8_t in[128] {}, out[256] {};
    memcpy(in + 20, settings, 12);
    put32(in + 4, AP_HAL::micros());
    // Initial construction uses the module worker's larger stack.
    if (invoke(fn, state, in, out, base) != 1 || out[0] != 1) {
        return false;
    }
    entry = fn; rw = base; owner = new_owner;
    memcpy(latest, out, 256);
    request_pending = response_ready = false;
    epoch++;
    return true;
}
void remove(void *old_owner)
{
    WITH_SEMAPHORE(gate);
    if (config_owner == old_owner) {
        config_owner = nullptr;
    }
    if (owner && owner == old_owner) {
        entry = nullptr; rw = nullptr; owner = nullptr;
        memset(latest, 0, sizeof(latest));
        latest[2] = ERROR_DETACHED;
        request_pending = response_ready = false;
        epoch++;
    }
}
void counters(uint32_t words[5])
{
    words[0] = calls; words[1] = rejected; words[2] = unavailable; words[3] = faults; words[4] = maximum_us;
}
void report(uint8_t output[256])
{
    WITH_SEMAPHORE(gate);
    memcpy(output, latest, 256);
}
bool snapshot(void *requester, uint32_t offset, uint8_t output[16])
{
    WITH_SEMAPHORE(gate);
    if (!requester || requester != owner || offset > sizeof(latest) - 16) {
        return false;
    }
    memcpy(output, latest + offset, 16);
    return true;
}
void Device::packet(const uint8_t *data)
{
    input[1] = data ? 1 : 2;
    input[18] = data ? size : 0;
    if (data) {
        memcpy(input + 20, data, size);
    }
    put32(input + 8, radio.last_irq_us);
    pending = true;
}
void Device::worker()
{
    radio.irq_callback = FUNCTOR_BIND_MEMBER(&Device::packet, void, const uint8_t *);
    while (true) {
        if (!pending) {
            radio.update();
        }
        uint8_t out[256] {};
        // Blocking acquisition lets the HAL mutex inherit the RF priority
        // while a lower-priority owner finishes its bounded mailbox operation.
        gate.take_blocking();
        if (!entry) {
            gate.give();
            if (generation) {
                radio.standby();
                generation = 0;
            }
            pending = false; memset(input, 0, sizeof(input));
            hal.scheduler->delay(5); continue;
        }
        const uint32_t current = epoch;
        if (generation != current) {
            memset(input, 0, sizeof(input));
            pending = false;
            generation = current;
            frequency = 0;
        }
        input[0] = 1;
        input[2] = radio.healthy();
        input[16] = uint8_t(radio.last_packet_rssi);
        input[17] = uint8_t(radio.last_packet_snr_raw);
#ifdef HAL_SX1280_BUTTON_PIN
        input[19] = hal.gpio->read(HAL_SX1280_BUTTON_PIN) == HAL_SX1280_BUTTON_ACTIVE_HIGH;
#endif
        put32(input + 4, AP_HAL::micros()); put32(input + 12, radio.irq_overruns());
        const bool ok = call(input, out);
        if (ok) {
            memcpy(latest, out, 256);
        }
        gate.give();
        if (!ok || current != epoch.load()) {
            radio.standby(); pending = false;
            hal.scheduler->delay(5); continue;
        }
        pending = false; input[1] = input[3] = input[18] = 0;
        const uint8_t actions = out[1];
        if (actions & 16) {
            radio.standby();
        }
        if (actions & 1) {
            input[3] = radio.begin() && radio.enable_irq_timestamps() ? 1 : 2;
        }
        if (actions & 2) {
            size = out[18]; frequency = get32(out + 8);
            const uint16_t crc = uint16_t(out[16]) | uint16_t(out[17]) << 8;
            const bool configured = out[19] ?
                                    radio.config_flrc(get32(out + 8), out[22], size, get32(out + 12), crc) :
                                    radio.config_lora(SX1280_LORA_BW_0800, out[20], out[21], get32(out + 8), out[22], out[23], size);
            radio.set_output_power(int8_t(out[24]));
            if (!configured) {
                input[3] = 2;
            }
        } else if (get32(out + 8) != 0 && get32(out + 8) != frequency && radio.healthy()) {
            // Cached in hardware executor, independent of protocol meaning.
            frequency = get32(out + 8);
            radio.set_frequency_reg(frequency);
        }
        if (actions & 4) {
            radio.rx();
        }
        if (actions & 8) {
            if (int32_t(AP_HAL::micros() - get32(out + 240)) > 0) {
                input[3] = 3;
            } else if (out[25] != size || !radio.tx(out + 26)) {
                input[3] = 2;
            } else {
                input[3] = 4;
                put32(input + 36, AP_HAL::micros());
            }
        }
        if (!input[3] || input[3] == 4) {
            const uint32_t deadline = get32(input + 4) + get32(out + 4);
            // Run deferred stream work only with room before the radio wakeup.
            if (int32_t(deadline - AP_HAL::micros()) > 500 && gate.take_nonblocking()) {
                if (entry && current == epoch.load() && request_pending) {
                    put32(request + 4, AP_HAL::micros());
                    response_ready = call(request, response);
                    request_pending = false;
                }
                gate.give();
            }
            const int32_t remaining = int32_t(deadline - AP_HAL::micros());
            (void)radio.wait_for_irq(remaining > 0 ? MIN(uint32_t(remaining), 5000U) : 0);
        }
    }
}
void update()
{
    auto *uart = AP_HAL::VirtualUARTDriver::get_uart(AP_RADIO_SX1280_UART);
    const uint32_t txspace = uart ? uart->txspace() : 0;
    const uint32_t rxspace = uart ? uart->rxspace() : 0;
    if (!gate.take_nonblocking()) {
        return;
    }
    if (!entry) {
        gate.give();
        return;
    }
    if (!started) {
        started = device.start();
        if (!started) {
            latest[2] = 7;
            gate.give();
            return;
        }
    }
    const uint32_t current = epoch;
    if (main_epoch != current) {
        main_epoch = current;
        down_size = up_ack = 0;
        drained = false;
        main_generation = 0;
    }
    uint8_t in[128] {}, out[256] {};
    in[0] = 2; in[1] = down_size; in[2] = up_ack; in[3] = drained;
    memcpy(in + 20, down, down_size);
    put32(in + 4, AP_HAL::micros()); put32(in + 84, main_generation);
    if (uart) {
        put32(in + 8, txspace);
        put32(in + 12, rxspace);
    }
    if (!response_ready) {
        if (!request_pending) {
            memcpy(request, in, sizeof(request));
            request_pending = true;
        }
        gate.give();
        return;
    }
    memcpy(out, response, sizeof(out));
    response_ready = false;
    gate.give();
    if (current != epoch.load()) {
        return;
    }
    if (main_generation != get32(out + 244)) {
        down_size = 0;
    }
    main_generation = get32(out + 244);
    if (out[42]) {
        down_size = 0;
    }
    up_ack = 0;
    if (uart) {
        if (out[3] & 16) {
            down_size = 0;
            uint8_t trash[120];
            drained = false;
            for (uint8_t i = 0; i < 8; i++) {
                if (uart->read_tx(trash, sizeof(trash)) < sizeof(trash)) {
                    drained = true;
                    break;
                }
            }
        } else {
            drained = false;
            if (!down_size && (out[3] & 32)) {
                down_size = uart->read_tx(down, sizeof(down));
            }
        }
        if (out[43]) {
            up_ack = uart->write_rx(out + 44, out[43]);
        }
    }
#ifdef HAL_SX1280_LED_PIN
    const bool on = out[3] & 4;
    hal.gpio->write(HAL_SX1280_LED_PIN, HAL_SX1280_LED_ACTIVE_HIGH ? on : !on);
#endif
}
}
#endif
