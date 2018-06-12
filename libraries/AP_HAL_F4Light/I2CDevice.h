/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <inttypes.h>
#include "AP_HAL_F4Light_Namespace.h"
#include "Scheduler.h"

#include <AP_HAL/HAL.h>
#include <AP_HAL_F4Light/HAL_F4Light_Class.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "Semaphores.h"

#include <i2c.h>
#include "tim_i2c.h"

#define MAX_I2C_DEVICES 10

using namespace F4Light;

#ifdef I2C_DEBUG
enum I2C_Log_State {
    I2C_START,
    I2C_ISR,
    I2C_STOP,
    I2C_ERR,
    I2C_FINISH,
};

typedef struct I2C_STATE {
    uint32_t time;
    uint8_t addr;
    uint8_t bus;
    uint8_t send_len;
    uint8_t recv_len;
    uint8_t ret;
    uint16_t cr1;
    uint16_t sr1;
    uint16_t sr2;
    uint16_t st_sr1;
    uint16_t st_sr2;
    uint8_t state;
    I2C_Log_State pos;
} I2C_State;
#endif


class F4Light::I2CDevice : public AP_HAL::I2CDevice {
public:

    I2CDevice(uint8_t bus, uint8_t address);
    
    ~I2CDevice();

    static void lateInit();

    /* AP_HAL::I2CDevice implementation */

    /* See AP_HAL::I2CDevice::set_address() */
    inline void set_address(uint8_t address) override { _address = address; }

    /* See AP_HAL::I2CDevice::set_retries() */
    inline void set_retries(uint8_t retries) override { _retries = retries; }


    /* AP_HAL::Device implementation */

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;


    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times);


    /* See AP_HAL::Device::set_speed() */
    inline bool set_speed(enum AP_HAL::Device::Speed speed) override { return true; };

    /* See AP_HAL::Device::get_semaphore() */
    inline F4Light::Semaphore *get_semaphore() override { return &_semaphores[_bus]; } // numbers from 0

    /* See AP_HAL::Device::register_periodic_callback() */
    inline AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, Device::PeriodicCb proc) override
    {   
        return Scheduler::register_timer_task(period_usec, proc, get_semaphore() );
    }


    inline bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override 
    {
        return Scheduler::adjust_timer_task(h, period_usec);
    }
    
    inline bool unregister_callback(PeriodicHandle h) override  { return Scheduler::unregister_timer_task(h); }

    void register_completion_callback(Handler h);

    inline void register_completion_callback(AP_HAL::MemberProc proc){
        Revo_handler r = { .mp=proc };
        register_completion_callback(r.h);
    }
    inline void register_completion_callback(AP_HAL::Proc proc){
        Revo_handler r = { .hp=proc };
        register_completion_callback(r.h);
    }

    inline uint32_t get_error_count() { return _lockup_count; }
    inline uint8_t  get_last_error() { return last_error; }
    inline uint8_t  get_last_error_state() { return last_error_state; }
    inline uint8_t  get_bus()  { return _bus; }
    inline uint8_t  get_addr() { return _address; }
    
    static inline uint8_t get_dev_count() { return dev_count; }
    static inline F4Light::I2CDevice * get_device(uint8_t i) { return devices[i]; }

    void do_bus_reset();

private:
    void init();

    uint32_t i2c_read(uint8_t addr, const uint8_t *tx_buff, uint8_t txlen, uint8_t *rx_buff, uint8_t rxlen);
    uint32_t i2c_write(uint8_t addr, const uint8_t *tx_buff, uint8_t len);
    void  isr_ev();
    uint32_t wait_stop_done(bool v);
    void finish_transfer();

    uint8_t  _bus;
    uint16_t _offs;
    uint8_t  _address;
    uint8_t  _retries;
    uint32_t _lockup_count;
    bool     _initialized;
    uint8_t  last_error;
    uint8_t  last_error_state;
    bool     _slow;
    bool     _failed;
    bool     need_reset;
    void     *_task;
    
    const i2c_dev *_dev;
    Soft_I2C *s_i2c; // per-bus instances

    static F4Light::Semaphore _semaphores[3]; // individual for each bus + softI2C
    static const timer_dev *   _timers[3];   // one timer per bus
    
    static F4Light::I2CDevice * devices[MAX_I2C_DEVICES]; // links to all created devices
    static uint8_t dev_count;
    static bool lateInitDone;
    
    Handler _completion_cb;

    uint8_t _state; // state of transfer for ISR
    volatile uint8_t _error; // error from ISR
    uint8_t _addr;      //      data for ISR
    const uint8_t *_tx_buff;
    uint8_t  _tx_len;
    uint8_t *_rx_buff;
    uint8_t  _rx_len;

    void _do_bus_reset();
    
#ifdef I2C_DEBUG
#define I2C_LOG_SIZE 99
    static I2C_State log[I2C_LOG_SIZE];
    static uint8_t log_ptr;
#endif
};

class F4Light::I2CDeviceManager : public AP_HAL::I2CDeviceManager {
    friend class F4Light::I2CDevice;

public:
    I2CDeviceManager() { }

    /* AP_HAL::I2CDeviceManager implementation */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, 
                                                uint8_t address,
                                                uint32_t bus_clock=400000,
                                                bool use_smbus = false,
                                                uint32_t timeout_ms=4) {

        // let's first check for existence of such device on same bus
        uint8_t n = I2CDevice::get_dev_count();

        for(uint8_t i=0; i<n; i++){
            I2CDevice * d = I2CDevice::get_device(i);
            if(d){
                if(d->get_bus() == bus && d->get_addr() == address) { // device already exists
                    return nullptr;
                }
            }
        }
    
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(
            new I2CDevice(bus, address)
        );
    }
};

