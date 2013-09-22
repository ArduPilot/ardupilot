/*
 * Console.h --- AP_HAL_SMACCM console driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __AP_HAL_SMACCM_CONSOLE_H__
#define __AP_HAL_SMACCM_CONSOLE_H__

#include <AP_HAL_SMACCM.h>

class SMACCM::SMACCMConsoleDriver : public AP_HAL::ConsoleDriver {
public:
    SMACCMConsoleDriver(AP_HAL::BetterStream* delegate);
    void init(void *arg);
    void backend_open();
    void backend_close();
    size_t backend_read(uint8_t *data, size_t len);
    size_t backend_write(const uint8_t *data, size_t len);

    int16_t available();
    int16_t txspace();
    int16_t read();

    size_t write(uint8_t c);
private:
    AP_HAL::BetterStream *_d;

    // Buffer implementation copied from the AVR HAL.
    struct Buffer {
        /* public methods:*/
        bool allocate(uint16_t size);
        bool push(uint8_t b);
        int16_t  pop();

        uint16_t bytes_free();
        uint16_t bytes_used();
    private:
        uint16_t _head, _tail; /* Head and tail indicies */
        uint16_t _mask;       /* Buffer size mask for index wrap */
        uint8_t *_bytes;      /* Pointer to allocated buffer */
    };

    Buffer _txbuf;
    Buffer _rxbuf;
    bool _user_backend;
};

#endif // __AP_HAL_SMACCM_CONSOLE_H__
