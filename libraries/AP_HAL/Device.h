/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
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

#include "AP_HAL_Namespace.h"
#include "utility/functor.h"
#include "AP_HAL_Boards.h"

#if CONFIG_HAL_BOARD != HAL_BOARD_QURT
// we need utility for std::move, but not on QURT due to a include error in hexagon SDK
#include <utility>
#endif

/*
 * This is an interface abstracting I2C and SPI devices
 */
class AP_HAL::Device {
public:
    enum BusType {
        BUS_TYPE_UNKNOWN = 0,
        BUS_TYPE_I2C     = 1,
        BUS_TYPE_SPI     = 2,
        BUS_TYPE_UAVCAN  = 3,
        BUS_TYPE_SITL    = 4,
        BUS_TYPE_MSP     = 5,
        BUS_TYPE_SERIAL  = 6,
        BUS_TYPE_WSPI    = 7,
    };

    enum Speed {
        SPEED_HIGH,
        SPEED_LOW,
    };

    // Used for comms with devices that support wide SPI
    // like quad spi
    struct CommandHeader {
        uint32_t  cmd; //Command phase data.
        uint32_t  cfg; //Transfer configuration field.
        uint32_t  addr; //Address phase data.
        uint32_t  alt; // Alternate phase data.
        uint32_t  dummy; // Number of dummy cycles to be inserted.
    };

    FUNCTOR_TYPEDEF(PeriodicCb, void);
    typedef void* PeriodicHandle;

    // Register Read Write Callback
    // returns: void parameters: register address, register data, register datasize, direction R:false, W:true
    FUNCTOR_TYPEDEF(RegisterRWCb, void, uint8_t, uint8_t*, uint32_t, bool);
    typedef void* RegisterRWHandle;

    FUNCTOR_TYPEDEF(BankSelectCb, bool, uint8_t);

    Device(enum BusType type)
    {
        _bus_id.devid_s.bus_type = type;
    }

    // return bus type
    enum BusType bus_type(void) const {
        return _bus_id.devid_s.bus_type;
    }

    // return bus number
    uint8_t bus_num(void) const {
        return _bus_id.devid_s.bus;
    }

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const {
        return _bus_id.devid;
    }

    // return address on bus
    uint8_t get_bus_address(void) const {
        return _bus_id.devid_s.address;
    }

    // set device type within a device class (eg. AP_COMPASS_TYPE_LSM303D)
    void set_device_type(uint8_t devtype);

    virtual ~Device() {
        delete[] _checked.regs;
    }

    /*
     * Change device address. Note that this is the 7 bit address, it
     * does not include the bit for read/write. Only works on I2C
     */
    virtual void set_address(uint8_t address) {};
    
    /*
     * Set the speed of future transfers. Depending on the bus the speed may
     * be shared for all devices on the same bus.
     *
     * Return: true if speed was successfully set or platform doesn't implement
     * it; false otherwise.
     */
    virtual bool set_speed(Speed speed)  = 0;

    /*
     * This does a single bus transaction which sends send_len bytes, 
     * then receives recv_len bytes back from the slave. Operation is
     * half-duplex independent of bus type.
     *
     * Return: true on a successful transfer, false on failure.
     */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) = 0;

    /*
     * Core transfer function. This does a single bus transaction which
     * sends len bytes and receives len bytes back from the slave.
     * On full-duplex buses (e.g. SPI), the transfer is full-duplex,
     * unlike `transfer`.
     * DMA-optimized on some implementations
     *
     * Return: true on a successful transfer, false on failure.
     */
    virtual bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) {
        return transfer(send_recv, len, send_recv, len);
    }

    /*
     * Sets the required flags before transaction starts
     * this is to be used by Wide SPI communication interfaces like
     * Dual/Quad/Octo SPI
     */
    virtual void set_cmd_header(const CommandHeader& cmd_hdr) {}

    /*
     * Sets up peripheral for execution in place mode
     * Only relevant for Wide SPI setup.
     */
    virtual bool enter_xip_mode(void** map_ptr) { return false; }
    virtual bool exit_xip_mode() { return false; }

    /**
     * Wrapper function over #transfer() to read recv_len registers, starting
     * by first_reg, into the array pointed by recv. The read flag passed to
     * #set_read_flag(uint8_t) is ORed with first_reg before performing the
     * transfer.
     *
     * Return: true on a successful transfer, false on failure.
     */
    bool read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len);

    /**
     * Wrapper function over #transfer() to write a byte to the register reg.
     * The transfer is done by sending reg and val in that order.
     *
     * Return: true on a successful transfer, false on failure.
     */
    bool write_register(uint8_t reg, uint8_t val, bool checked=false);
    
    /*
     * Sets a callback to be called when a register is read or written.
     */
    virtual void set_register_rw_callback(RegisterRWCb register_rw_callback) {
        _register_rw_callback = register_rw_callback;
    }

    /**
     * Wrapper function over #transfer() to call bank selection callback
     * and then invoke the transfer call
     *
     * Return: true on a successful transfer, false on failure.
     */
    bool transfer_bank(uint8_t bank, const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len);

    /**
     * Wrapper function over #transfer_bank() to read recv_len registers, starting
     * by first_reg, into the array pointed by recv. The read flag passed to
     * #set_read_flag(uint8_t) is ORed with first_reg before performing the
     * transfer.
     *
     * Return: true on a successful transfer, false on failure.
     */
    bool read_bank_registers(uint8_t bank, uint8_t first_reg, uint8_t *recv, uint32_t recv_len);

    /**
     * Wrapper function over #transfer_bank() to write a byte to the register reg.
     * The transfer is done by sending reg and val in that order.
     *
     * Return: true on a successful transfer, false on failure.
     */
    bool write_bank_register(uint8_t bank, uint8_t reg, uint8_t val, bool checked=false);

    /**
     * set a value for a checked register in a bank
     */
    void set_checked_register(uint8_t bank, uint8_t reg, uint8_t val);

    /**
     * set a value for a checked register
     */
    void set_checked_register(uint8_t reg, uint8_t val);

    /**
     * setup for register value checking. Frequency is how often to check registers. If set to 10 then
     * every 10th call to check_next_register will check a register
     */
    bool setup_checked_registers(uint8_t num_regs, uint8_t frequency=10);

    /**
     * check next register value for correctness. Return false if value is incorrect
     * or register checking has not been setup
     */
    bool check_next_register(void);

    // checked registers
    struct checkreg {
        uint8_t bank;
        uint8_t regnum;
        uint8_t value;
    };
    
    /**
     * check next register value for correctness, with return of
     * failure value. Return false if value is incorrect or register
     * checking has not been setup
     */
    bool check_next_register(struct checkreg &fail);
    
    /**
     * Wrapper function over #transfer() to read a sequence of bytes from
     * device. No value is written, differently from the #read_registers()
     * method and hence doesn't include the read flag set by #set_read_flag()
     */
    bool read(uint8_t *recv, uint32_t recv_len)
    {
        return transfer(nullptr, 0, recv, recv_len);
    }

    /*
     * Get the semaphore for the bus this device is in.  This is intended for
     * drivers to use during initialization phase only.
     */
    virtual AP_HAL::Semaphore *get_semaphore() = 0;

    /*
     * Register a periodic callback for this bus. All callbacks on the
     * same bus are made from the same thread with lock already taken. In
     * other words, the callback is not executed on the main thread (or the
     * thread which registered the callback), but in a separate per-bus
     * thread.
     *
     * After registering the periodic callback, the other functions should not
     * be used anymore from other contexts. If it really needs to be done, the
     * lock must be taken.
     *
     * Return: A handle for this periodic callback. To cancel the callback
     * call #unregister_callback() or return false on the callback.
     */
    virtual PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb) = 0;

    /*
     * Adjust the time for the periodic callback registered with
     * #register_periodic_callback. Note that the time will be re-calculated
     * from the moment this call is made and expire after @period_usec.
     *
     * Return: true if periodic callback was successfully adjusted, false otherwise.
     */
    virtual bool adjust_periodic_callback(PeriodicHandle h, uint32_t period_usec) = 0;

    /*
     * Cancel a periodic callback on this bus.
     *
     * Return: true if callback was successfully unregistered, false
     * otherwise.
     */
    virtual bool unregister_callback(PeriodicHandle h) { return false; }

    /*
     * Sets a bank_select callback to be used for bank selection during register check
     */
    virtual void setup_bankselect_callback(BankSelectCb bank_select) {
        _bank_select = bank_select;
    }

    /*
     * Sets a bank_select callback to be used for bank selection during register check
     */
    virtual void deregister_bankselect_callback() {
        _bank_select = nullptr;
    }


    /*
        allows to set callback that will be called after DMA transfer complete.
        if this callback is set then any read/write operation will return directly after transfer setup and
        bus semaphore must not be released until register_completion_callback(0) called from callback itself
    */
    virtual void register_completion_callback(AP_HAL::MemberProc proc) {}
    virtual void register_completion_callback(AP_HAL::Proc proc) {}
    
    /*
     * support for direct control of SPI chip select. Needed for
     * devices with unusual SPI transfer patterns that include
     * specific delays
     */
    virtual bool set_chip_select(bool set) { return false; }

    /**
     * Some devices connected on the I2C or SPI bus require a bit to be set on
     * the register address in order to perform a read operation. This sets a
     * flag to be used by #read_registers(). The flag's default value is zero.
     */
    void set_read_flag(uint8_t flag);

    /**
     * make a bus id given bus type, bus number, bus address and
     * device type This is for use by devices that do not use one of
     * the standard HAL Device types, such as UAVCAN devices
     */
    static uint32_t make_bus_id(enum BusType bus_type, uint8_t bus, uint8_t address, uint8_t devtype);

    /**
     * return a new bus ID for the same bus connection but a new device type.
     * This is used for auxiliary bus connections
     */
    static uint32_t change_bus_id(uint32_t old_id, uint8_t devtype);

    /**
     * return bus ID with a new devtype
     */
    uint32_t get_bus_id_devtype(uint8_t devtype) const;

    /**
     * get bus type
     */
    static enum BusType devid_get_bus_type(uint32_t dev_id);

    static uint8_t devid_get_bus(uint32_t dev_id);

    static uint8_t devid_get_address(uint32_t dev_id);

    static uint8_t devid_get_devtype(uint32_t dev_id);


    /* set number of retries on transfers */
    virtual void set_retries(uint8_t retries) {};

protected:
    uint8_t _read_flag = 0;

    /*
      broken out device elements. The bitfields are used to keep
      the overall value small enough to fit in a float accurately,
      which makes it possible to transport over the MAVLink
      parameter protocol without loss of information.
     */
    struct DeviceStructure {
        enum BusType bus_type : 3;
        uint8_t bus: 5;    // which instance of the bus type
        uint8_t address;   // address on the bus (eg. I2C address)
        uint8_t devtype;   // device class specific device type
    };

    union DeviceId {
        struct DeviceStructure devid_s;
        uint32_t devid;
    };

    union DeviceId _bus_id;

    // set device address (eg. i2c bus address or spi CS)
    void set_device_address(uint8_t address) {
        _bus_id.devid_s.address = address;
    }

    // set device bus number
    void set_device_bus(uint8_t bus) {
        _bus_id.devid_s.bus = bus;
    }

private:
    BankSelectCb _bank_select;
    RegisterRWCb _register_rw_callback;
    struct {
        uint8_t n_allocated;
        uint8_t n_set;
        uint8_t next;
        uint8_t frequency;
        uint8_t counter;
        struct checkreg last_reg_fail;
        struct checkreg *regs;
    } _checked;
};
