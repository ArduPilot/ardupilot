#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <string.h>

#include "hardware/dma.h"
#include "hardware/spi.h"

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

// Track DMA channels for each bus (initialized channels, -1 if not initialized)
static int spi_dma_tx_channels[sizeof(spi_buses) / sizeof(spi_buses[0])] = {-1};
static int spi_dma_rx_channels[sizeof(spi_buses) / sizeof(spi_buses[0])] = {-1};

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

    // Initialize DMA channels for this bus
    int tx_ch = bd.dma_ch_tx;
    int rx_ch = bd.dma_ch_rx;

    // If AUTO (-1), claim unused channels
    if (tx_ch == -1) {
        tx_ch = dma_claim_unused_channel(true);
    }
    if (rx_ch == -1) {
        rx_ch = dma_claim_unused_channel(true);
    }

    // Configure TX DMA channel (will be fully configured per transfer)
    dma_channel_config tx_config = dma_channel_get_default_config(tx_ch);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_dreq(&tx_config, spi_get_dreq(bd.host, true));  // TX DREQ
    channel_config_set_read_increment(&tx_config, true);   // Read from buffer
    channel_config_set_write_increment(&tx_config, false); // Write to SPI FIFO
    dma_channel_set_config(tx_ch, &tx_config, false);

    // Configure RX DMA channel (will be fully configured per transfer)
    dma_channel_config rx_config = dma_channel_get_default_config(rx_ch);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_dreq(&rx_config, spi_get_dreq(bd.host, false)); // RX DREQ
    channel_config_set_read_increment(&rx_config, false);  // Read from SPI FIFO
    channel_config_set_write_increment(&rx_config, true);  // Write to buffer
    dma_channel_set_config(rx_ch, &rx_config, false);

    spi_dma_tx_channels[bus] = tx_ch;
    spi_dma_rx_channels[bus] = rx_ch;

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
    int tx_ch = spi_dma_tx_channels[_desc.bus];
    int rx_ch = spi_dma_rx_channels[_desc.bus];

    configure_bus();

    // Assert CS
    gpio_put(_desc.cs, 0);

    // Use DMA for transfers if channels are available
    if (tx_ch >= 0 && rx_ch >= 0 && len > 4) {
        // Use DMA for larger transfers (threshold: 4 bytes)
        // For smaller transfers, blocking is more efficient due to DMA setup overhead

        // Prepare dummy buffer for RX-only or TX-only transfers
        // Use stack buffer for small transfers, max 256 bytes
        uint8_t dummy_buf[256];
        const uint8_t *tx_buf = send;
        uint8_t *rx_buf = recv;

        if (send && recv) {
            // Full duplex: use provided buffers
            tx_buf = send;
            rx_buf = recv;
        } else if (send && !recv) {
            // TX only: send data, discard RX
            tx_buf = send;
            if (len <= sizeof(dummy_buf)) {
                rx_buf = dummy_buf;
            } else {
                // Too large for stack, fall back to blocking
                spi_write_blocking(bd.host, send, len);
                gpio_put(_desc.cs, 1);
                return true;
            }
        } else if (!send && recv) {
            // RX only: send 0xFF, receive data
            if (len <= sizeof(dummy_buf)) {
                memset(dummy_buf, 0xFF, len);
                tx_buf = dummy_buf;
                rx_buf = recv;
            } else {
                // Too large for stack, fall back to blocking
                spi_read_blocking(bd.host, 0xFF, recv, len);
                gpio_put(_desc.cs, 1);
                return true;
            }
        }

        // Abort any ongoing transfers
        dma_channel_abort(tx_ch);
        dma_channel_abort(rx_ch);

        // Wait for channels to be ready
        while (dma_channel_is_busy(tx_ch) || dma_channel_is_busy(rx_ch)) {
            tight_loop_contents();
        }

        // Configure and start TX channel
        dma_channel_set_read_addr(tx_ch, tx_buf, false);
        dma_channel_set_write_addr(tx_ch, &spi_get_hw(bd.host)->dr, false);
        dma_channel_set_transfer_count(tx_ch, len, false);

        // Configure and start RX channel
        dma_channel_set_read_addr(rx_ch, &spi_get_hw(bd.host)->dr, false);
        dma_channel_set_write_addr(rx_ch, rx_buf, false);
        dma_channel_set_transfer_count(rx_ch, len, false);

        // Start both channels simultaneously
        dma_start_channel_mask((1u << tx_ch) | (1u << rx_ch));

        // Wait for both channels to complete
        while (dma_channel_is_busy(tx_ch) || dma_channel_is_busy(rx_ch)) {
            tight_loop_contents();
        }

    } else {
        // Fallback to blocking transfers for small transfers or if DMA not available
        if (send && recv) {
            spi_write_read_blocking(bd.host, send, recv, len);
        } else if (send && !recv) {
            spi_write_blocking(bd.host, send, len);
        } else if (!send && recv) {
            // send 0xFF while reading
            spi_read_blocking(bd.host, 0xFF, recv, len);
        }
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

