
#ifndef __AP_HAL_FLYMAPLE_CONSOLE_H__
#define __AP_HAL_FLYMAPLE_CONSOLE_H__

#include <AP_HAL_FLYMAPLE.h>

class AP_HAL_FLYMAPLE_NS::FLYMAPLEConsoleDriver : public AP_HAL::ConsoleDriver {
public:
    FLYMAPLEConsoleDriver(void* baseuartdriver);
    void init(void* machtnichts);
    void backend_open();
    void backend_close();
    size_t backend_read(uint8_t *data, size_t len);
    size_t backend_write(const uint8_t *data, size_t len);

    int16_t available();
    int16_t txspace();
    int16_t read();

    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);
private:
    AP_HAL::UARTDriver* _base_uart;
};

#endif // __AP_HAL_FLYMAPLE_CONSOLE_H__
