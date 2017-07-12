#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_SENSORHUB_ENABLED

#include "Protocol.h"
#include "AP_SensorHub_IO.h"

#include <AP_HAL/utility/RingBuffer.h>

#include <unistd.h>

class AP_SensorHub;

class AP_SensorHub_IO_FileDescriptor : public AP_SensorHub_IO {
public:

    void registerInput(int inputFd) {
        _inputFd = inputFd;
    }

    void registerOutput(int outputFd) {
        _outputFd = outputFd;
    }

    virtual void read();

    virtual bool write(Packet::packet_t *packet, size_t len);

private:
    bool _isInputInitialized() {
        return _shub && _inputFd > 0;
    }

    bool _isOutputInitialized() {
        return _shub && _outputFd > 0;
    }
    // NOTE:
    // Copter (px4-v2):
    // 1000/s = accel @ 1kHz Invensense + 1000/s LSM9DS0
    // 1000/s = gyro @ 1kHz  Invensense + 760/s LSM9DS0
    // 10/s = baro @ 10Hz
    // 75/s = compass @ 75Hz HMC5843
    // 50/s = gps @ 50Hz
    // ~3895 packets/s

    // NOTE: read() is called at ~1kHz. We must buffer accordingly.
    ByteBuffer recvBuffer {10*Packet::MAX_PACKET_LEN};
    uint8_t dataBuffer[Packet::MAX_PACKET_LEN];
    uint8_t writeBuffer[Packet::MAX_PACKET_LEN];

    int _inputFd;
    int _outputFd;
};
#endif
