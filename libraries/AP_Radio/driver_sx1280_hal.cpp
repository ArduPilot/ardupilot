/*
  Low-level SX1280 SPI/GPIO transaction driver - see driver_sx1280_hal.h for notes
  on provenance and scope.
*/
#include "driver_sx1280_hal.h"

#if AP_RADIO_SX1280_ENABLED

#include <string.h>

extern const AP_HAL::HAL &hal;

AP_SX1280_HAL::AP_SX1280_HAL()
{
}

bool AP_SX1280_HAL::init(void)
{
    dev = hal.spi->get_device("sx1280");
    io_ok = (bool)dev;
    return io_ok;
}

void AP_SX1280_HAL::reset(void)
{
    hal.gpio->write(HAL_SX1280_RESET_PIN, 0);
    hal.scheduler->delay(50);
    hal.gpio->write(HAL_SX1280_RESET_PIN, 1);
    hal.scheduler->delay(50); // BUSY takes longer to go low than our wait_on_busy() timeout
    wait_on_busy();
}

bool AP_SX1280_HAL::wait_on_busy(void)
{
    const uint32_t timeout_us = 1000;
    const uint32_t start_us = AP_HAL::micros();
    while (hal.gpio->read(HAL_SX1280_BUSY_PIN) != 0) {
        if (AP_HAL::micros() - start_us > timeout_us) {
            io_ok = false;
            return false;
        }
    }
    return true;
}

bool AP_SX1280_HAL::irq_pending(void) const
{
    return hal.gpio->read(HAL_SX1280_DIO1_PIN) != 0;
}

void AP_SX1280_HAL::write_command(SX1280_RadioCommands_t command, uint8_t val, uint32_t busy_delay_us)
{
    write_command(command, &val, 1, busy_delay_us);
}

void AP_SX1280_HAL::write_command(SX1280_RadioCommands_t command, const uint8_t *buffer, uint8_t size, uint32_t busy_delay_us)
{
    uint8_t out[size + 1];
    out[0] = (uint8_t)command;
    memcpy(&out[1], buffer, size);

    if (!dev) {
        return;
    }
    WITH_SEMAPHORE(dev->get_semaphore());
    if (!io_ok || !wait_on_busy()) {
        return;
    }
    io_ok = dev->transfer(out, size + 1, nullptr, 0);
    hal.scheduler->delay_microseconds(busy_delay_us);
}

uint8_t AP_SX1280_HAL::read_command(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    uint8_t frame[size + 2];
    memset(frame, 0, sizeof(frame));
    frame[0] = (uint8_t)command;

    if (!dev) {
        memset(buffer, 0, size);
        return 0;
    }
    WITH_SEMAPHORE(dev->get_semaphore());
    if (!io_ok || !wait_on_busy()) {
        memset(buffer, 0, size);
        return 0;
    }
    io_ok = dev->transfer_fullduplex(frame, frame, sizeof(frame));
    if (!io_ok) {
        memset(buffer, 0, size);
        return 0;
    }
    memcpy(buffer, &frame[2], size);
    return frame[0] & SX1280_STATUS_MASK;
}

void AP_SX1280_HAL::write_register(uint16_t address, uint8_t value)
{
    write_register(address, &value, 1);
}

void AP_SX1280_HAL::write_register(uint16_t address, const uint8_t *buffer, uint8_t size)
{
    uint8_t out[size + 3];
    out[0] = SX1280_RADIO_WRITE_REGISTER;
    out[1] = (uint8_t)((address & 0xFF00) >> 8);
    out[2] = (uint8_t)(address & 0x00FF);
    memcpy(&out[3], buffer, size);

    if (!dev) {
        return;
    }
    WITH_SEMAPHORE(dev->get_semaphore());
    if (!io_ok || !wait_on_busy()) {
        return;
    }
    io_ok = dev->transfer(out, size + 3, nullptr, 0);
    hal.scheduler->delay_microseconds(15);
}

uint8_t AP_SX1280_HAL::read_register(uint16_t address)
{
    uint8_t value = 0;
    read_register(address, &value, 1);
    return value;
}

void AP_SX1280_HAL::read_register(uint16_t address, uint8_t *buffer, uint8_t size)
{
    const uint8_t header[4] = {
        SX1280_RADIO_READ_REGISTER,
        (uint8_t)((address & 0xFF00) >> 8),
        (uint8_t)(address & 0x00FF),
        0x00, // dummy byte before register data
    };

    if (!dev) {
        memset(buffer, 0, size);
        return;
    }
    WITH_SEMAPHORE(dev->get_semaphore());
    if (!io_ok || !wait_on_busy()) {
        memset(buffer, 0, size);
        return;
    }
    io_ok = dev->transfer(header, sizeof(header), buffer, size);
    if (!io_ok) {
        memset(buffer, 0, size);
    }
}

void AP_SX1280_HAL::write_buffer(uint8_t offset, const uint8_t *buffer, uint8_t size)
{
    uint8_t out[size + 2];
    out[0] = SX1280_RADIO_WRITE_BUFFER;
    out[1] = offset;
    memcpy(&out[2], buffer, size);

    if (!dev) {
        return;
    }
    WITH_SEMAPHORE(dev->get_semaphore());
    if (!io_ok || !wait_on_busy()) {
        return;
    }
    io_ok = dev->transfer(out, size + 2, nullptr, 0);
    hal.scheduler->delay_microseconds(15);
}

void AP_SX1280_HAL::read_buffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
    const uint8_t header[3] = { SX1280_RADIO_READ_BUFFER, offset, 0x00 };

    if (!dev) {
        memset(buffer, 0, size);
        return;
    }
    WITH_SEMAPHORE(dev->get_semaphore());
    if (!io_ok || !wait_on_busy()) {
        memset(buffer, 0, size);
        return;
    }
    io_ok = dev->transfer(header, sizeof(header), buffer, size);
    if (!io_ok) {
        memset(buffer, 0, size);
    }
}

#endif  // AP_RADIO_SX1280_ENABLED
