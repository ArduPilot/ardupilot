#pragma once

#include "AP_PackAuth_config.h"

#if AP_PACKAUTH_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Param/AP_Param.h>

class AP_PackAuth
{
public:
    AP_PackAuth();
    CLASS_NO_COPY(AP_PackAuth);

    // start the authenticator thread (idempotent, call after parameters are loaded)
    void init();

    // arming gate: true only if the pack proved its identity and the thread is alive;
    // otherwise fills buf with the reason
    bool gate_check(char *buf, size_t len) const;

#if AP_PACKAUTH_PARAMS_ENABLED
    static const struct AP_Param::GroupInfo var_info[];
#endif

private:
    void thread();
    void step();
    void find_device();
    bool probe(uint8_t bus, uint8_t addr);
    void challenge();
    bool alive() const;
#if AP_PACKAUTH_DEBUG
    void debug_print();
#endif

#if AP_PACKAUTH_PARAMS_ENABLED
    AP_Int8 _bus;
    AP_Int8 _addr;
    int8_t  cfg_bus() const  { return _bus; }
    uint8_t cfg_addr() const { return (uint8_t)_addr.get(); }
#else
    int8_t  cfg_bus() const  { return AP_PACKAUTH_BUS_DEFAULT; }
    uint8_t cfg_addr() const { return AP_PACKAUTH_ADDR_DEFAULT; }
#endif

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    bool _started = false;
    bool _thread_failed = false;
    volatile bool _ok = false;               // written by the thread only
    volatile uint32_t _beat_ms = 0;          // thread heartbeat
    uint32_t _ok_ms = 0;
    uint32_t _last_scan_ms = 0;
    uint32_t _last_try_ms = 0;
    uint16_t _ok_serial = 0;
    int32_t  _serial = -1;
    int8_t   _bus_found = -1;
    uint8_t  _last_status = 0xFF;
    const char *_why = "starting";
    uint16_t _n_sent = 0, _n_ok = 0, _n_bad = 0, _n_xfer_fail = 0, _n_timeout = 0;
#if AP_PACKAUTH_DEBUG
    uint32_t _dbg_ms = 0;
#endif
};

#endif  // AP_PACKAUTH_ENABLED