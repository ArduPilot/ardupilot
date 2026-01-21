#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <string.h>

using namespace RP;

extern const AP_HAL::HAL& hal;

// Bus and device tables generated from hwdef.dat
#ifdef HAL_RP2350_SPI_BUSES
static SPIBusDesc spi_buses[] = { HAL_RP2350_SPI_BUSES };
#endif

#ifdef HAL_RP2350_SPI_DEVICES
static SPIDeviceDesc spi_devices[] = { HAL_RP2350_SPI_DEVICES };
#endif

// Track which buses have been initialised
static bool spi_bus_initialised[sizeof(spi_buses) / sizeof(spi_buses[0])] = {};

static void init_bus(uint8_t bus)
{
    if (bus >= (uint8_t)(sizeof(spi_buses) / sizeof(spi_buses[0]))) {
        return;
    }
    if (spi_bus_initialised[bus]) {
        return;
    }

    const SPIBusDesc &bd = spi_buses[bus];

    // Basic SPI setup using Pico SDK
    // The baud rate will be adjusted per-device before each transfer.
    spi_init(bd.host, 1 * MHZ);

    gpio_set_function(bd.mosi, GPIO_FUNC_SPI);
    gpio_set_function(bd.miso, GPIO_FUNC_SPI);
    gpio_set_function(bd.sclk, GPIO_FUNC_SPI);

    spi_bus_initialised[bus] = true;
}

SPIDevice::SPIDevice(const SPIDeviceDesc &desc) :
    _desc(desc),
    _speed(AP_HAL::Device::SPEED_LOW),
    _current_baud(desc.lspeed)
{
    init_bus(_desc.bus);
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    _speed = speed;
    _current_baud = (speed == AP_HAL::Device::SPEED_HIGH) ? _desc.hspeed : _desc.lspeed;
    return true;
}

void SPIDevice::configure_bus()
{
    if (_desc.bus >= (uint8_t)(sizeof(spi_buses) / sizeof(spi_buses[0]))) {
        return;
    }

    const SPIBusDesc &bd = spi_buses[_desc.bus];

    // Configure SPI format for this device: 8 bits, CPOL/CPHA from mode
    bool cpol = (_desc.mode == 2 || _desc.mode == 3);
    bool cpha = (_desc.mode == 1 || _desc.mode == 3);

    spi_set_baudrate(bd.host, _current_baud);
    spi_set_format(bd.host,
                   8,
                   cpol ? SPI_CPOL_1 : SPI_CPOL_0,
                   cpha ? SPI_CPHA_1 : SPI_CPHA_0,
                   SPI_MSB_FIRST);

    // Configure CS as a normal GPIO (ArduPilot controls it explicitly)
    gpio_init(_desc.cs);
    gpio_set_dir(_desc.cs, GPIO_OUT);
    gpio_put(_desc.cs, 1);
}

bool SPIDevice::do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    if (len == 0) {
        return true;
    }

    if (!_semaphore.check_owner()) {
        return false;
    }

    if (_desc.bus >= (uint8_t)(sizeof(spi_buses) / sizeof(spi_buses[0]))) {
        return false;
    }

    const SPIBusDesc &bd = spi_buses[_desc.bus];

    configure_bus();

    // Assert CS
    gpio_put(_desc.cs, 0);

    if (send && recv) {
        spi_write_read_blocking(bd.host, send, recv, len);
    } else if (send && !recv) {
        spi_write_blocking(bd.host, send, len);
    } else if (!send && recv) {
        // send 0xFF while reading
        spi_read_blocking(bd.host, 0xFF, recv, len);
    }

    // Deassert CS
    gpio_put(_desc.cs, 1);

    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    // Callers should prefer transfer_fullduplex(), but we handle the
    // generic semantics similarly to other HALs.

    if ((send_len == recv_len && send == recv) || !send || !recv) {
        // simplest cases, needed for DMA-style transfers
        const uint32_t len = recv_len ? recv_len : send_len;
        return do_transfer(send, recv, len);
    }

    // Need a combined buffer
    uint32_t len = send_len + recv_len;
    uint8_t buf[len];

    if (send_len > 0 && send) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }

    bool ok = do_transfer(buf, buf, len);
    if (ok && recv_len > 0 && recv) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return ok;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                                    uint32_t len)
{
    return do_transfer(send, recv, len);
}

AP_HAL::Device::PeriodicHandle
SPIDevice::register_periodic_callback(uint32_t,
                                      AP_HAL::Device::PeriodicCb)
{
    // No dedicated device-thread support yet on RP HAL.
    return nullptr;
}

bool SPIDevice::clock_pulse(uint32_t len)
{
    if (len == 0) {
        return true;
    }

    if (!_semaphore.check_owner()) {
        return false;
    }

    if (_desc.bus >= (uint8_t)(sizeof(spi_buses) / sizeof(spi_buses[0]))) {
        return false;
    }

    const SPIBusDesc &bd = spi_buses[_desc.bus];

    configure_bus();

    // Keep CS high (inactive) for clock pulses
    gpio_put(_desc.cs, 1);

    // Send dummy bytes to generate clock pulses
    uint8_t dummy = 0xFF;
    for (uint32_t i = 0; i < len; i++) {
        spi_write_blocking(bd.host, &dummy, 1);
    }

    return true;
}

AP_HAL::SPIDevice *
SPIDeviceManager::get_device_ptr(const char *name)
{
#ifdef HAL_RP2350_SPI_DEVICES
    for (uint8_t i = 0; i < (uint8_t)(sizeof(spi_devices) / sizeof(spi_devices[0])); i++) {
        if (strcmp(spi_devices[i].name, name) == 0) {
            return NEW_NOTHROW SPIDevice(spi_devices[i]);
        }
    }
#endif
    // Unknown device name
    return nullptr;
}

