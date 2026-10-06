#define __EXPORT __attribute__ ((visibility ("default")))

#ifndef __cplusplus
#error "C++ should be defined!!!"
#endif

#include <stdint.h>

// Functions that are called by the SLPI LINK server into AP client
extern "C" {
    // Called by the SLPI LINK server to initialize and start AP
    int slpi_link_client_init(void) __EXPORT;

    // Called by the SLPI LINK server when there is a new message for AP
    int slpi_link_client_receive(const uint8_t *data, int data_len_in_bytes) __EXPORT;
}

// Functions in SLPI LINK server that are called by AP client into DSP
extern "C" {
    // Send a message to the applications processor
    int sl_client_send_data(const uint8_t *data, int data_len_in_bytes);

    void sl_client_register_fatal_error_cb(void (*func)(void));

    // Interrupt callback registration
    int sl_client_register_interrupt_callback(int (*func)(int, void*, void*), void* arg);

    // Get DSP CPU utilization (0 - 100)
    int sl_client_get_cpu_utilization(void);

    // I2C interface API
    int sl_client_config_i2c_bus(uint8_t bus_number, uint8_t address, uint32_t frequency);
    void sl_client_set_address_i2c_bus(int fd, uint8_t address);
    int sl_client_i2c_transfer(int fd, const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len);

    // SPI interface API
    int sl_client_spi_transfer(int fd, const uint8_t *send, uint8_t *recv, const unsigned len);
    int sl_client_config_spi_bus(void);

    // UART interface API
    int sl_client_config_uart(uint8_t port_number, uint32_t speed);
    int sl_client_uart_write(int fd, const char *data, const unsigned data_len);
    int sl_client_uart_read(int fd, char *buffer, const unsigned buffer_len);
}

// IDs for serial ports
#define QURT_UART_GPS 6
#define QURT_UART_RCIN 7
#define QURT_UART_ESC_IO 2 // UART for the ESC or IO board that bridges to ESC
