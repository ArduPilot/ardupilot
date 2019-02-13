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

#if HAL_USE_I2C == TRUE

#include "Scheduler.h"
#include "hwdef/common/stm32_util.h"

#include "ch.h"
#include "hal.h"

static const struct I2CInfo {
    struct I2CDriver *i2c;
    uint8_t dma_channel_rx;
    uint8_t dma_channel_tx;
    ioline_t scl_line;
    ioline_t sda_line;
} I2CD[] = { HAL_I2C_DEVICE_LIST };

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(I2CD)];

#ifndef HAL_I2C_BUS_BASE
#define HAL_I2C_BUS_BASE 0
#endif

// default to 100kHz clock for maximum reliability. This can be
// changed in hwdef.dat
#ifndef HAL_I2C_MAX_CLOCK
#define HAL_I2C_MAX_CLOCK 100000
#endif

// values calculated with STM32CubeMX tool, PCLK=54MHz
#define HAL_I2C_F7_100_TIMINGR 0x20404768
#define HAL_I2C_F7_400_TIMINGR 0x6000030D

/*
  enable clear (toggling SCL) on I2C bus timeouts which leave SDA stuck low
 */
#ifndef HAL_I2C_CLEAR_ON_TIMEOUT
#define HAL_I2C_CLEAR_ON_TIMEOUT 1
#endif

// get a handle for DMA sharing DMA channels with other subsystems
void I2CBus::dma_init(void)
{
    chMtxObjectInit(&dma_lock);
    dma_handle = new Shared_DMA(I2CD[busnum].dma_channel_tx, I2CD[busnum].dma_channel_rx, 
                                FUNCTOR_BIND_MEMBER(&I2CBus::dma_allocate, void, Shared_DMA *),
                                FUNCTOR_BIND_MEMBER(&I2CBus::dma_deallocate, void, Shared_DMA *));    
}

// Clear Bus to avoid bus lockup
void I2CBus::clear_all()
{
    for (uint8_t i=0; i<ARRAY_SIZE(I2CD); i++) {
        clear_bus(i);
    }
}

/*
  clear a stuck bus (bus held by a device that is holding SDA low) by
  clocking out pulses on SCL to let the device complete its
  transaction
 */
void I2CBus::clear_bus(uint8_t busidx)
{
    const struct I2CInfo &info = I2CD[busidx];
    const iomode_t mode_saved = palReadLineMode(info.scl_line);
    palSetLineMode(info.scl_line, PAL_MODE_OUTPUT_PUSHPULL);
    for(uint8_t j = 0; j < 20; j++) {
        palToggleLine(info.scl_line);
        hal.scheduler->delay_microseconds(10);
    }
    palSetLineMode(info.scl_line, mode_saved);
}

/*
  read SDA on a bus, to check if it may be stuck
 */
uint8_t I2CBus::read_sda(uint8_t busidx)
{
    const struct I2CInfo &info = I2CD[busidx];
    const iomode_t mode_saved = palReadLineMode(info.sda_line);
    palSetLineMode(info.sda_line, PAL_MODE_INPUT);
    uint8_t ret = palReadLine(info.sda_line);
    palSetLineMode(info.sda_line, mode_saved);
    return ret;
}

// setup I2C buses
I2CDeviceManager::I2CDeviceManager(void)
{
    for (uint8_t i=0; i<ARRAY_SIZE(I2CD); i++) {
        businfo[i].busnum = i;
        businfo[i].dma_init();
        /*
          setup default I2C config. As each device is opened we will
          drop the speed to be the minimum speed requested
         */
        businfo[i].busclock = HAL_I2C_MAX_CLOCK;
#if defined(STM32F7)
        if (businfo[i].busclock <= 100000) {
            businfo[i].i2ccfg.timingr = HAL_I2C_F7_100_TIMINGR;
            businfo[i].busclock = 100000;
        } else {
            businfo[i].i2ccfg.timingr = HAL_I2C_F7_400_TIMINGR;
            businfo[i].busclock = 400000;
        }
#else
        businfo[i].i2ccfg.op_mode = OPMODE_I2C;
        businfo[i].i2ccfg.clock_speed = businfo[i].busclock;
        if (businfo[i].i2ccfg.clock_speed <= 100000) {
            businfo[i].i2ccfg.duty_cycle = STD_DUTY_CYCLE;
        } else {
            businfo[i].i2ccfg.duty_cycle = FAST_DUTY_CYCLE_2;
        }
#endif
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
    if (bus_clock < bus.busclock) {
#if defined(STM32F7)
        if (bus_clock <= 100000) {
            bus.i2ccfg.timingr = HAL_I2C_F7_100_TIMINGR;
            bus.busclock = 100000;
        }
#else
        bus.i2ccfg.clock_speed = bus_clock;
        bus.busclock = bus_clock;
        if (bus_clock <= 100000) {
            bus.i2ccfg.duty_cycle = STD_DUTY_CYCLE;
        }
#endif
        hal.console->printf("I2C%u clock %ukHz\n", busnum, unsigned(bus.busclock/1000));
    }
}

I2CDevice::~I2CDevice()
{
#if 0
    printf("I2C device bus %u address 0x%02x closed\n", 
           (unsigned)bus.busnum, (unsigned)_address);
#endif
    free(pname);
}

/*
  allocate DMA channel, nothing to do, as we don't keep the bus active between transactions
 */
void I2CBus::dma_allocate(Shared_DMA *ctx)
{
}

/*
  deallocate DMA channel
 */
void I2CBus::dma_deallocate(Shared_DMA *)
{
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (!bus.semaphore.check_owner()) {
        hal.console->printf("I2C: not owner of 0x%x\n", (unsigned)get_bus_id());
        return false;
    }
    
#if defined(STM32F7)
    if (_use_smbus) {
        bus.i2ccfg.cr1 |= I2C_CR1_SMBHEN;
    } else {
        bus.i2ccfg.cr1 &= ~I2C_CR1_SMBHEN;
    }
#else
    if (_use_smbus) {
        bus.i2ccfg.op_mode = OPMODE_SMBUS_HOST;
    } else {
        bus.i2ccfg.op_mode = OPMODE_I2C;
    }
#endif

    if (_split_transfers) {
        /*
          splitting the transfer() into two pieces avoids a stop condition
          with SCL low which is not supported on some devices (such as
          LidarLite blue label)
        */
        if (send && send_len) {
            if (!_transfer(send, send_len, nullptr, 0)) {
                return false;
            }
        }
        if (recv && recv_len) {
            if (!_transfer(nullptr, 0, recv, recv_len)) {
                return false;
            }
        }
    } else {
        // combined transfer
        if (!_transfer(send, send_len, recv, recv_len)) {
            return false;
        }
    }

    return true;
}

bool I2CDevice::_transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    i2cAcquireBus(I2CD[bus.busnum].i2c);

    bus.bouncebuffer_setup(send, send_len, recv, recv_len);
    
    for(uint8_t i=0 ; i <= _retries; i++) {
        int ret;
        // calculate a timeout as twice the expected transfer time, and set as min of 4ms
        uint32_t timeout_ms = 1+2*(((8*1000000UL/bus.busclock)*MAX(send_len, recv_len))/1000);
        timeout_ms = MAX(timeout_ms, _timeout_ms);

        // we get the lock and start the bus inside the retry loop to
        // allow us to give up the DMA channel to an SPI device on
        // retries
        bus.dma_handle->lock();

        i2cStart(I2CD[bus.busnum].i2c, &bus.i2ccfg);
        osalDbgAssert(I2CD[bus.busnum].i2c->state == I2C_READY, "i2cStart state");
        
        if(send_len == 0) {
            ret = i2cMasterReceiveTimeout(I2CD[bus.busnum].i2c, _address, recv, recv_len, chTimeMS2I(timeout_ms));
        } else {
            ret = i2cMasterTransmitTimeout(I2CD[bus.busnum].i2c, _address, send, send_len,
                                           recv, recv_len, chTimeMS2I(timeout_ms));
        }

        i2cStop(I2CD[bus.busnum].i2c);
        osalDbgAssert(I2CD[bus.busnum].i2c->state == I2C_STOP, "i2cStart state");
        
        bus.dma_handle->unlock();
        
        if (ret == MSG_OK) {
            bus.bouncebuffer_finish(send, recv, recv_len);
            i2cReleaseBus(I2CD[bus.busnum].i2c);
            return true;
        }
#if HAL_I2C_CLEAR_ON_TIMEOUT
        if (ret == MSG_TIMEOUT && I2CBus::read_sda(bus.busnum) == 0) {
            I2CBus::clear_bus(bus.busnum);
        }
#endif
    }
    bus.bouncebuffer_finish(send, recv, recv_len);
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
    if (bus >= ARRAY_SIZE(I2CD)) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(nullptr);
    }
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms));
    return dev;
}

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
    return ((1U << ARRAY_SIZE(I2CD)) - 1) << HAL_I2C_BUS_BASE;
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    // assume first bus is internal
    return get_bus_mask() & HAL_I2C_INTERNAL_MASK;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    // assume first bus is internal
    return get_bus_mask() & ~HAL_I2C_INTERNAL_MASK;
}

#endif // HAL_USE_I2C
