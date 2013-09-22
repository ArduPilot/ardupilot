
#ifndef __AP_HAL_PX4_CONSOLE_H__
#define __AP_HAL_PX4_CONSOLE_H__

#include <AP_HAL_PX4.h>
#include <AP_HAL_PX4_Namespace.h>

class PX4::PX4ConsoleDriver : public AP_HAL::ConsoleDriver {
public:
    PX4ConsoleDriver();
    void init(void* machtnichts);
    void backend_open();
    void backend_close();
    size_t backend_read(uint8_t *data, size_t len);
    size_t backend_write(const uint8_t *data, size_t len);

    int16_t available();
    int16_t txspace();
    int16_t read();

    size_t write(uint8_t c);

private:
    PX4UARTDriver *_uart;
};

#endif // __AP_HAL_PX4_CONSOLE_H__
