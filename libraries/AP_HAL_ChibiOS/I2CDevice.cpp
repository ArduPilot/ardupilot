/*
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
#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "Util.h"
#include "Scheduler.h"

#include "ch.h"
#include "hal.h"

static const struct I2CInfo {
    struct I2CDriver *i2c;
    uint8_t dma_channel_rx;
    uint8_t dma_channel_tx;
} I2CD[] = { HAL_I2C_DEVICE_LIST };

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE_SIMPLE(I2CD)];

#ifndef HAL_I2C_BUS_BASE
#define HAL_I2C_BUS_BASE 0
#endif

// default to 100kHz clock for maximum reliability. This can be
// changed in hwdef.dat
#ifndef HAL_I2C_MAX_CLOCK
#define HAL_I2C_MAX_CLOCK 100000
#endif

// get a handle for DMA sharing DMA channels with other subsystems
void I2CBus::dma_init(void)
{
    dma_handle = new Shared_DMA(I2CD[busnum].dma_channel_tx, I2CD[busnum].dma_channel_rx, 
                                FUNCTOR_BIND_MEMBER(&I2CBus::dma_allocate, void),
                                FUNCTOR_BIND_MEMBER(&I2CBus::dma_deallocate, void));    
}

// Clear Bus to avoid bus lockup
void I2CBus::clear_all()
{
#if defined(HAL_GPIO_PIN_I2C1_SCL) && defined(HAL_I2C1_SCL_AF)
    clear_bus(HAL_GPIO_PIN_I2C1_SCL, HAL_I2C1_SCL_AF);
#endif

#if defined(HAL_GPIO_PIN_I2C2_SCL) && defined(HAL_I2C2_SCL_AF)
    clear_bus(HAL_GPIO_PIN_I2C1_SCL, HAL_I2C1_SCL_AF);
#endif

#if defined(HAL_GPIO_PIN_I2C3_SCL) && defined(HAL_I2C3_SCL_AF)
    clear_bus(HAL_GPIO_PIN_I2C1_SCL, HAL_I2C1_SCL_AF);
#endif

#if defined(HAL_GPIO_PIN_I2C4_SCL) && defined(HAL_I2C4_SCL_AF)
    clear_bus(HAL_GPIO_PIN_I2C1_SCL, HAL_I2C1_SCL_AF);
#endif
}

//This code blocks!
void I2CBus::clear_bus(ioline_t scl_line, uint8_t scl_af)
{
    //send dummy clock
    palSetLineMode(scl_line, PAL_MODE_OUTPUT_PUSHPULL);
    for(int i = 0; i < 20; i++) {
        palToggleLine(scl_line);
        hal.scheduler->delay_microseconds(200);
    }
    palSetLineMode(scl_line, PAL_MODE_ALTERNATE(scl_af) | PAL_STM32_OSPEED_MID2 | PAL_STM32_OTYPE_OPENDRAIN);
}

// setup I2C buses
I2CDeviceManager::I2CDeviceManager(void)
{
    for (uint8_t i=0; i<ARRAY_SIZE_SIMPLE(I2CD); i++) {
        businfo[i].busnum = i;
        businfo[i].dma_init();
        /*
          setup default I2C config. As each device is opened we will
          drop the speed to be the minimum speed requested
         */
        businfo[i].i2ccfg.op_mode = OPMODE_I2C;
        businfo[i].i2ccfg.clock_speed = HAL_I2C_MAX_CLOCK;
        if (businfo[i].i2ccfg.clock_speed <= 100000) {
            businfo[i].i2ccfg.duty_cycle = STD_DUTY_CYCLE;
        } else {
            businfo[i].i2ccfg.duty_cycle = FAST_DUTY_CYCLE_2;
        }
    }
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    _retries(2),
    _address(address),
    _use_smbus(use_smbus),
    _timeout_ms(timeout_ms),
    bus(I2CDeviceManager::businfo[busnum])
{
    set_device_bus(busnum+HAL_I2C_BUS_BASE);
    set_device_address(address);
    asprintf(&pname, "I2C:%u:%02x",
             (unsigned)busnum, (unsigned)address);
    if (bus_clock < bus.i2ccfg.clock_speed) {
        bus.i2ccfg.clock_speed = bus_clock;
        hal.console->printf("I2C%u clock %ukHz\n", busnum, unsigned(bus_clock/1000));
        if (bus_clock <= 100000) {
            bus.i2ccfg.duty_cycle = STD_DUTY_CYCLE;
        }
    }
}

I2CDevice::~I2CDevice()
{
    printf("I2C device bus %u address 0x%02x closed\n", 
           (unsigned)bus.busnum, (unsigned)_address);
    free(pname);
}

/*
  allocate DMA channel
 */
void I2CBus::dma_allocate(void)
{
    if (!i2c_started) {
        osalDbgAssert(I2CD[busnum].i2c->state == I2C_STOP, "i2cStart state");
        i2cStart(I2CD[busnum].i2c, &i2ccfg);
        osalDbgAssert(I2CD[busnum].i2c->state == I2C_READY, "i2cStart state");
        i2c_started = true;
    }
}

/*
  deallocate DMA channel
 */
void I2CBus::dma_deallocate(void)
{
    if (i2c_started) {
        osalDbgAssert(I2CD[busnum].i2c->state == I2C_READY, "i2cStart state");
        i2cStop(I2CD[busnum].i2c);
        osalDbgAssert(I2CD[busnum].i2c->state == I2C_STOP, "i2cStart state");
        i2c_started = false;
    }
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (!bus.semaphore.check_owner()) {
        hal.console->printf("I2C: not owner of 0x%x\n", (unsigned)get_bus_id());
        return false;
    }
    
    bus.dma_handle->lock();

    if (_use_smbus) {
        bus.i2ccfg.op_mode = OPMODE_SMBUS_HOST;
    } else {
        bus.i2ccfg.op_mode = OPMODE_I2C;
    }
    
    if (_split_transfers) {
        /*
          splitting the transfer() into two pieces avoids a stop condition
          with SCL low which is not supported on some devices (such as
          LidarLite blue label)
        */
        if (send && send_len) {
            if (!_transfer(send, send_len, nullptr, 0)) {
                bus.dma_handle->unlock();
                return false;
            }
        }
        if (recv && recv_len) {
            if (!_transfer(nullptr, 0, recv, recv_len)) {
                bus.dma_handle->unlock();
                return false;
            }
        }
    } else {
        // combined transfer
        if (!_transfer(send, send_len, recv, recv_len)) {
            bus.dma_handle->unlock();
            return false;
        }
    }

    bus.dma_handle->unlock();
    return true;
}

bool I2CDevice::_transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    uint8_t *recv_buf = recv;
    const uint8_t *send_buf = send;

    bus.bouncebuffer_setup(send_buf, send_len, recv_buf, recv_len);

    i2cAcquireBus(I2CD[bus.busnum].i2c);
    
    for(uint8_t i=0 ; i <= _retries; i++) {
        int ret;
        // calculate a timeout as twice the expected transfer time, and set as min of 4ms
        uint32_t timeout_ms = 1+2*(((8*1000000UL/bus.i2ccfg.clock_speed)*MAX(send_len, recv_len))/1000);
        timeout_ms = MAX(timeout_ms, _timeout_ms);
        bus.i2c_active = true;
        osalDbgAssert(I2CD[bus.busnum].i2c->state == I2C_READY, "i2cStart state");
        if(send_len == 0) {
            ret = i2cMasterReceiveTimeout(I2CD[bus.busnum].i2c, _address, recv_buf, recv_len, MS2ST(timeout_ms));
        } else {
            ret = i2cMasterTransmitTimeout(I2CD[bus.busnum].i2c, _address, send_buf, send_len,
                                           recv_buf, recv_len, MS2ST(timeout_ms));
        }
        bus.i2c_active = false;
        if (ret != MSG_OK) {
            //restart the bus
            osalDbgAssert(I2CD[bus.busnum].i2c->state == I2C_READY || I2CD[bus.busnum].i2c->state == I2C_LOCKED, "i2cStart state");
            i2cStop(I2CD[bus.busnum].i2c);
            osalDbgAssert(I2CD[bus.busnum].i2c->state == I2C_STOP, "i2cStart state");
            i2cStart(I2CD[bus.busnum].i2c, &bus.i2ccfg);
            osalDbgAssert(I2CD[bus.busnum].i2c->state == I2C_READY, "i2cStart state");
        } else {
            osalDbgAssert(I2CD[bus.busnum].i2c->state == I2C_READY, "i2cStart state");
            if (recv_buf != recv) {
                memcpy(recv, recv_buf, recv_len);
            }
            i2cReleaseBus(I2CD[bus.busnum].i2c);
            return true;
        }
    }
    i2cReleaseBus(I2CD[bus.busnum].i2c);
    return false;
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    return false;
}

    
/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}
    

/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address,
                             uint32_t bus_clock,
                             bool use_smbus,
                             uint32_t timeout_ms)
{
    bus -= HAL_I2C_BUS_BASE;
    if (bus >= ARRAY_SIZE_SIMPLE(I2CD)) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(nullptr);
    }
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms));
    return dev;
}
