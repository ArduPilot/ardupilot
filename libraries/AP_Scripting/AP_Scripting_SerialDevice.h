#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_SERIALDEVICE_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>

#ifndef AP_SCRIPTING_SERIALDEVICE_NUM_PORTS
#define AP_SCRIPTING_SERIALDEVICE_NUM_PORTS 3
#endif

class AP_Scripting;

class AP_Scripting_SerialDevice
{
public:
    /* Do not allow copies */
    CLASS_NO_COPY(AP_Scripting_SerialDevice);

    AP_Scripting_SerialDevice() {}

    AP_Int8 enable;

    void init(void);
    void clear(void);

public:
    class Port : public AP_SerialManager::RegisteredPort {
    public:
        friend class AP_Scripting_SerialDevice;
        void init(void);
        void clear(void);

        size_t device_write(const uint8_t *buffer, size_t size);
        ssize_t device_read(uint8_t *buffer, uint16_t count);
        uint32_t device_available(void);

    private:
        bool is_initialized() override {
            return true;
        }
        bool tx_pending() override {
            return false;
        }

        bool init_buffers(const uint32_t size_rx, const uint32_t size_tx);

        uint32_t txspace() override;
        void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
        size_t _write(const uint8_t *buffer, size_t size) override;
        ssize_t _read(uint8_t *buffer, uint16_t count) override;
        uint32_t _available() override;
        void _end() override {}
        void _flush() override {}
        bool _discard_input() override;

        ByteBuffer *readbuffer;
        ByteBuffer *writebuffer;
        uint32_t last_size_tx;
        uint32_t last_size_rx;

        HAL_Semaphore sem;
    };

    Port ports[AP_SCRIPTING_SERIALDEVICE_NUM_PORTS];
};

#endif  // AP_SCRIPTING_SERIALDEVICE_ENABLED
