#include <assert.h>
#include <stdlib.h>

#include "AuxiliaryBus.h"

AuxiliaryBusSlave::AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr,
                                     uint8_t instance)
    : _bus(bus)
    , _addr(addr)
    , _instance(instance)
{
}

AuxiliaryBusSlave::~AuxiliaryBusSlave()
{
}

AuxiliaryBus::AuxiliaryBus(AP_InertialSensor_Backend &backend, uint8_t max_slaves, uint32_t devid)
    : _max_slaves(max_slaves)
    , _ins_backend(backend)
    , _devid(devid)
{
    _slaves = (AuxiliaryBusSlave**) calloc(max_slaves, sizeof(AuxiliaryBusSlave*));
}

AuxiliaryBus::~AuxiliaryBus()
{
    for (int i = _n_slaves - 1; i >= 0; i--) {
        delete _slaves[i];
    }
    free(_slaves);
}

/*
 * Get the next available slave for the sensor exposing this AuxiliaryBus.
 * If a new slave cannot be registered or instantiated, `nullptr` is returned.
 * Otherwise a new slave is returned, but it's not registered (and therefore
 * not owned by the AuxiliaryBus).
 *
 * After using the slave, if it's not registered for a periodic read it must
 * be destroyed.
 *
 * @addr: the address of this slave in the bus
 *
 * Return a new slave if successful or `nullptr` otherwise.
 */
AuxiliaryBusSlave *AuxiliaryBus::request_next_slave(uint8_t addr)
{
    if (_n_slaves == _max_slaves)
        return nullptr;

    AuxiliaryBusSlave *slave = _instantiate_slave(addr, _n_slaves);
    if (!slave)
        return nullptr;

    return slave;
}

/*
 * Register a periodic read. This should be called after the slave sensor is
 * already configured and the only thing the master needs to do is to copy a
 * set of registers from the slave to its own registers.
 *
 * The sample rate is hard-coded, depending on the sensor that exports this
 * AuxiliaryBus.
 *
 * After this call the AuxiliaryBusSlave is owned by this object and should
 * not be destroyed. A typical call chain to use a sensor in an AuxiliaryBus
 * is (error checking omitted for brevity):
 *
 *      AuxiliaryBusSlave *slave = bus->request_next_slave(addr);
 *      slave->passthrough_read(WHO_AM_I, buf, 1);
 *      slave->passthrough_write(...);
 *      slave->passthrough_write(...);
 *      ...
 *      bus->register_periodic_read(slave, SAMPLE_START_REG, SAMPLE_SIZE);
 *
 * @slave: the AuxiliaryBusSlave already configured to be in continuous mode
 * @reg:   the first register of the block to use in each periodic transfer
 * @size:  the block size, usually the size of the sample multiplied by the
 *         number of axes in each sample.
 *
 * Return 0 on success or < 0 on error.
 */
int AuxiliaryBus::register_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                         uint8_t size)
{
    assert(slave->_instance == _n_slaves);
    assert(_n_slaves < _max_slaves);

    int r = _configure_periodic_read(slave, reg, size);
    if (r < 0)
        return r;

    slave->_sample_reg_start = reg;
    slave->_sample_size = size;
    slave->_registered = true;
    _slaves[_n_slaves++] = slave;

    return 0;
}
