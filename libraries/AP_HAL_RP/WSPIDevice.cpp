#include "WSPIDevice.h"
#include "hardware/flash.h"
#include "pico/mutex.h"

#if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)

using namespace RP;

WSPIDevice::WSPIDevice(SemaphoreHandle_t lock) : _lock(lock) {
    // Initialize the default header
    memset(&_current_cmd_hdr, 0, sizeof(_current_cmd_hdr));
}

WSPIDeviceManager::WSPIDeviceManager() {
    _flash_mutex = xSemaphoreCreateMutex();
}

AP_HAL::OwnPtr<AP_HAL::WSPIDevice> WSPIDeviceManager::get_device(const char *name) {
    if (strcmp(name, "qspi_flash") != 0) {
        return nullptr;
    }
    return AP_HAL::OwnPtr<AP_HAL::WSPIDevice>(new RP::WSPIDevice(_flash_mutex));
}

void WSPIDevice::set_cmd_header(const CommandHeader& cmd_hdr) {
    _current_cmd_hdr = cmd_hdr;
}

bool WSPIDevice::is_busy() {
    // On RP2350 flash_do_cmd is blocking, so after exiting transfer the device is always free
    // But we can check the flash status register (W25Qxx)
    uint8_t status_reg_cmd = 0x05; // Read Status Register-1
    uint8_t status;
    
    // Call transfer to check the BUSY bit in the memory itself
    transfer(&status_reg_cmd, 1, &status, 1);
    
    return (status & 0x01); // Bit 0 - BUSY
}

bool WSPIDevice::transfer(const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len) {
    if (xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    // For RP2350, Flash operations require disabling interrupts
    // and suspending the second core if it uses XIP
    uint32_t ints = save_and_disable_interrupts();

#if PICO_MULTICORE
    multicore_lockout_start_blocking();
#endif

    bool success = true;

    // Form the command.
    // ArduPilot sends instructions either via set_cmd_header or in the send buffer itself.

    uint8_t opcode = _current_cmd_hdr.cmd;

    // If opcode is not set in the header, take the first byte from send
    if (opcode == 0 && send_len > 0) {
        opcode = send[0];
        send++;
        send_len--;
    }

    // Use the low-level SDK API to work directly with Flash
    // flash_do_cmd is a standard Pico SDK function that executes a command on the SSI bus
    // It automatically handles the transfer of the opcode and subsequent data.

    if (opcode != 0) {
        // Note: RP2350 SDK expects a complete data packet (cmd + data)
        // We combine header and data if necessary.
        // For ArduPilot JEDEC ID (0x9F) it looks like this:
        flash_do_cmd(&opcode, recv, recv_len);
    } else {
        success = false;
    }

    // Resume the second core and interrupts
#if PICO_MULTICORE
    multicore_lockout_end_blocking();
#endif
    restore_interrupts(ints);

    // Clear the header for the next transaction
    memset(&_current_cmd_hdr, 0, sizeof(_current_cmd_hdr));

    xSemaphoreGive(_lock);
    return success;
}
#endif // #if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)
