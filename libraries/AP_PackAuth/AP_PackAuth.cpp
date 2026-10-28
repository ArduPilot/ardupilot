#include "AP_PackAuth.h"

#if AP_PACKAUTH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "smartpack_proto.h"
#if __has_include("smartpack_key.h")
#include "smartpack_key.h"
#else
#error "Copy libraries/AP_PackAuth/smartpack_key.h.example to smartpack_key.h and set your own key"
#endif

extern const AP_HAL::HAL& hal;

#define SBS_REG_SERIAL   0x1C
#define PKA_SCAN_MS      2000    // rescan period while the pack isn't found
#define PKA_RETRY_MS     1000    // gap between challenges
#define PKA_TIMEOUT_MS   1000    // time the pack has to answer
#define PKA_REAUTH_MS   10000    // while disarmed, re-challenge this often
#define PKA_EXPIRE_MS   30000    // drop trust if no good answer for this long
#define PKA_BEAT_MS      2000    // thread must beat within this

#if AP_PACKAUTH_PARAMS_ENABLED
const AP_Param::GroupInfo AP_PackAuth::var_info[] = {
    // @Param: BUS
    // @DisplayName: Pack authenticator I2C bus
    // @Description: I2C bus of the pack authenticator. -1 scans all external buses.
    // @Range: -1 3
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("BUS", 1, AP_PackAuth, _bus, AP_PACKAUTH_BUS_DEFAULT),

    // @Param: ADDR
    // @DisplayName: Pack authenticator I2C address
    // @Description: I2C address of the pack authenticator
    // @Range: 0 127
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("ADDR", 2, AP_PackAuth, _addr, AP_PACKAUTH_ADDR_DEFAULT),

    AP_GROUPEND
};
#endif

AP_PackAuth::AP_PackAuth()
{
#if AP_PACKAUTH_PARAMS_ENABLED
    AP_Param::setup_object_defaults(this, var_info);
#endif
}

void AP_PackAuth::init()
{
    if (_started) {
        return;
    }
    _started = true;
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PackAuth::thread, void),
                                      "packauth", 2560, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        _thread_failed = true;     // gate stays closed
    }
}

bool AP_PackAuth::alive() const
{
    return _beat_ms != 0 && (AP_HAL::millis() - _beat_ms) < PKA_BEAT_MS;
}

bool AP_PackAuth::gate_check(char *buf, size_t len) const
{
    if (_ok && alive()) {
        return true;
    }
    const char *why = _why;
    if (_thread_failed)  { why = "thread failed"; }
    else if (!_started)  { why = "not started"; }
    else if (!alive())   { why = (_beat_ms == 0) ? "starting" : "task stalled"; }
    else if (!_dev)      { why = "scanning I2C"; }
    hal.util->snprintf(buf, len, "Pack auth: %s", why);
    return false;
}

void AP_PackAuth::thread()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(10);
    }
    while (true) {
        _beat_ms = AP_HAL::millis();
        step();
        hal.scheduler->delay(100);
    }
}

void AP_PackAuth::step()
{
    const uint32_t now = AP_HAL::millis();

#if AP_PACKAUTH_DEBUG
    debug_print();
#endif

    if (!_dev) {
        if (now - _last_scan_ms < PKA_SCAN_MS) {
            return;
        }
        _last_scan_ms = now;
        find_device();
        if (!_dev) {
            _why = "scanning I2C";
            return;
        }
    }

    if (_ok) {
        if (hal.util->get_soft_armed()) {
            return;                           // never disturb an armed vehicle
        }
        if (now - _ok_ms > PKA_EXPIRE_MS) {
            _ok = false;                      // pack stopped answering
            _why = "pack stopped answering";
        } else if (now - _ok_ms < PKA_REAUTH_MS) {
            return;                           // proven recently
        }
    }

    if (now - _last_try_ms < PKA_RETRY_MS) {
        return;
    }
    _last_try_ms = now;
    challenge();
}

void AP_PackAuth::find_device()
{
    const uint8_t addr = cfg_addr();
    const int8_t want = cfg_bus();

    if (want >= 0) {
        // fixed bus: trust the setting, the challenge will show if nothing answers
        _bus_found = want;
        _dev = hal.i2c_mgr->get_device(want, addr, 100000, true, 20);
        return;
    }

    const uint32_t mask = hal.i2c_mgr->get_bus_mask_external();
    for (uint8_t b = 0; b < 8; b++) {
        if (!(mask & (1U << b))) {
            continue;
        }
        if (probe(b, addr)) {
            _bus_found = b;
            _dev = hal.i2c_mgr->get_device(b, addr, 100000, true, 20);
            return;
        }
    }
}

// does something at addr on this bus answer our STATUS register with a valid value?
bool AP_PackAuth::probe(uint8_t bus, uint8_t addr)
{
    auto d = hal.i2c_mgr->get_device(bus, addr, 100000, true, 20);
    if (!d) {
        return false;
    }
    d->set_retries(2);
    WITH_SEMAPHORE(d->get_semaphore());
    uint8_t st = 0xFF;
    return d->read_registers(SMARTPACK_REG_STATUS, &st, 1) && st <= SMARTPACK_ST_READY;
}

void AP_PackAuth::challenge()
{
    // 1. pack serial: standard SBS SerialNumber(), plain 2-byte read, no PEC
    uint8_t sb[2];
    {
        WITH_SEMAPHORE(_dev->get_semaphore());
        if (!_dev->read_registers(SBS_REG_SERIAL, sb, sizeof(sb))) {
            _n_xfer_fail++;
            _why = "no answer on I2C";
            return;
        }
    }
    const uint16_t serial = (uint16_t)sb[0] | ((uint16_t)sb[1] << 8);
    _serial = serial;
    if (_ok && serial != _ok_serial) {
        _ok = false;                          // a different pack: trust nothing
    }

    // 2. fresh random challenge
    uint8_t nonce[SMARTPACK_NONCE_LEN];
    if (!hal.util->get_random_vals(nonce, sizeof(nonce))) {
        for (uint8_t i = 0; i < sizeof(nonce); i++) {
            nonce[i] = get_random16() & 0xFF;
        }
    }
    uint8_t pkt[1 + SMARTPACK_NONCE_LEN];
    pkt[0] = SMARTPACK_REG_NONCE;
    memcpy(&pkt[1], nonce, sizeof(nonce));
    {
        WITH_SEMAPHORE(_dev->get_semaphore());
        if (!_dev->transfer(pkt, sizeof(pkt), nullptr, 0)) {
            _n_xfer_fail++;
            _why = "nonce write failed";
            return;
        }
    }
    _n_sent++;

    // 3. wait for READY; the pack never stretches the clock
    uint8_t resp[SMARTPACK_RESPONSE_LEN];
    bool ready = false;
    const uint32_t t0 = AP_HAL::millis();
    while (AP_HAL::millis() - t0 < PKA_TIMEOUT_MS) {
        hal.scheduler->delay(20);
        _beat_ms = AP_HAL::millis();
        WITH_SEMAPHORE(_dev->get_semaphore());
        uint8_t st = 0xFF;
        if (!_dev->read_registers(SMARTPACK_REG_STATUS, &st, 1)) {
            _n_xfer_fail++;
            continue;
        }
        _last_status = st;
        if (st == SMARTPACK_ST_READY) {
            ready = _dev->read_registers(SMARTPACK_REG_RESPONSE, resp, sizeof(resp));
            if (!ready) {
                _n_xfer_fail++;
            }
            break;
        }
    }
    if (!ready) {
        _n_timeout++;
        _why = "pack not responding";
        return;
    }

    // 4. verify without early exit
    uint8_t expected[SMARTPACK_RESPONSE_LEN];
    smartpack_compute_response(SMARTPACK_KEY, nonce, serial, expected);
    uint8_t diff = 0;
    for (uint8_t i = 0; i < sizeof(resp); i++) {
        diff |= expected[i] ^ resp[i];
    }
    if (diff == 0) {
        _ok_serial = serial;
        _ok_ms = AP_HAL::millis();
        _ok = true;
        _n_ok++;
        _why = "authenticated";
    } else {
        _ok = false;                          // a wrong answer revokes trust
        _n_bad++;
        _why = "wrong response";
    }
}

#if AP_PACKAUTH_DEBUG
void AP_PackAuth::debug_print()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _dbg_ms < 5000 || hal.util->get_soft_armed()) {
        return;
    }
    _dbg_ms = now;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PKA bus=%d ok=%u ser=%ld %s",
                  (int)_bus_found, _ok ? 1U : 0U, (long)_serial, _why);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PKA tx=%u ok=%u bad=%u xf=%u to=%u st=%u",
                  (unsigned)_n_sent, (unsigned)_n_ok, (unsigned)_n_bad,
                  (unsigned)_n_xfer_fail, (unsigned)_n_timeout, (unsigned)_last_status);
}
#endif

#endif  // AP_PACKAUTH_ENABLED