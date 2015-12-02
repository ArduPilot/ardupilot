/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
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

class AuxiliaryBus;
class AP_InertialSensor_Backend;

namespace AP_HAL {
    class Semaphore;
}

class AuxiliaryBusSlave
{
    friend class AuxiliaryBus;

public:
    virtual ~AuxiliaryBusSlave();

    /*
     * Read a block of registers from the slave. This is a one-time read. Must
     * be implemented by the sensor exposing the AuxiliaryBus.
     *
     * This method cannot be called after the periodic read is configured
     * since the registers for the periodic read may be shared with the
     * passthrough reads.
     *
     * @reg: the first register of the block to use in this one time transfer
     * @buf: buffer in which to write the values read
     * @size: the buffer size
     *
     * Return the number of bytes read on success or < 0 on error
     */
    virtual int passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size) = 0;

    /*
     * Write a single value into a register of this slave. Must be implemented
     * by the sensor exposing the AuxiliaryBus.
     *
     * This method cannot be called after the periodic read is configured
     * since the registers for the periodic read may be shared with the
     * passthrough writes.
     *
     * @reg: the register to use in this one time transfer
     * @val: the value to write
     *
     * Return the number of bytes written on success or < 0 on error
     */
    virtual int passthrough_write(uint8_t reg, uint8_t val) = 0;

    /*
     * Read the block of registers that were read from the slave on the last
     * time a periodic read occurred.
     *
     * This method must be called after the periodic read is configured and
     * the buffer must be large enough to accomodate the size configured.
     *
     * @buf: buffer in which to write the values read
     *
     * Return the number of bytes read on success or < 0 on error
     */
    virtual int read(uint8_t *buf) = 0;

protected:
    /* Only AuxiliaryBus is able to create a slave */
    AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr, uint8_t instance);

    AuxiliaryBus &_bus;
    uint8_t _addr = 0;
    uint8_t _instance = 0;

    uint8_t _sample_reg_start = 0;
    uint8_t _sample_size = 0;

    bool _registered = false;
};

class AuxiliaryBus
{
    friend class AP_InertialSensor_Backend;

public:
    AP_InertialSensor_Backend &get_backend() { return _ins_backend; }

    AuxiliaryBusSlave *request_next_slave(uint8_t addr);
    int register_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg, uint8_t size);

    /*
     * Get the semaphore needed to call methods on the bus this sensor is on.
     * Internally no locks are taken and it's the caller's duty to lock and
     * unlock the bus as needed.
     *
     * This method must be implemented by the sensor exposing the
     * AuxiliaryBus.
     *
     * Return the semaphore used protect transfers on the bus
     */
    virtual AP_HAL::Semaphore *get_semaphore() = 0;

protected:
    /* Only AP_InertialSensor_Backend is able to create a bus */
    AuxiliaryBus(AP_InertialSensor_Backend &backend, uint8_t max_slaves);
    virtual ~AuxiliaryBus();

    virtual AuxiliaryBusSlave *_instantiate_slave(uint8_t addr, uint8_t instance) = 0;
    virtual int _configure_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                         uint8_t size) = 0;

    uint8_t _n_slaves = 0;
    const uint8_t _max_slaves;
    AuxiliaryBusSlave **_slaves;
    AP_InertialSensor_Backend &_ins_backend;
};
