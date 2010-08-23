// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-
//
// Interrupt-driven serial transmit/receive library.
//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
// Receive and baudrate calculations derived from the Arduino 
// HardwareSerial driver:
//
//      Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
//
// Transmit algorithm inspired by work:
//
//      Code Jose Julio and Jordi Munoz. DIYDrones.com
//
//      This library is free software; you can redistribute it and/or
//      modify it under the terms of the GNU Lesser General Public
//      License as published by the Free Software Foundation; either
//      version 2.1 of the License, or (at your option) any later
//      version.
//
//      This library is distributed in the hope that it will be
//      useful, but WITHOUT ANY WARRANTY; without even the implied
//      warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//      PURPOSE.  See the GNU Lesser General Public License for more
//      details.
//
//      You should have received a copy of the GNU Lesser General
//      Public License along with this library; if not, write to the
//      Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
//      Boston, MA 02110-1301 USA
//

//
// Note that this library does not pre-declare drivers for serial
// ports; the user must explicitly create drivers for the ports they
// wish to use.  This is less friendly than the stock Arduino driver,
// but it saves 256 bytes for every unused port.
//
// To adjust the transmit/receive buffer sizes, change the size of the
// 'bytes' member in the RXBuffer and TXBuffer structures.
//

#ifndef FastSerial_h
#define FastSerial_h

#include <inttypes.h>
#include <Print.h>

// disable the stock Arduino serial driver
#define HardwareSerial_h

class FastSerial : public Print {
public:
        FastSerial(uint8_t portNumber = 0);

        // Serial API
        void            begin(long baud);
        void            end(void);
        uint8_t         available(void);
        int             read(void);
        void            flush(void);
        void            write(uint8_t c);
        void            write(const uint8_t *buffer, int count);
        using Print::write;

        // Interrupt methods
        void            receive(uint8_t c);
        void            transmit(void);

private:
        // register accessors
        volatile uint8_t *_ubrrh;
        volatile uint8_t *_ubrrl;
        volatile uint8_t *_ucsra;
        volatile uint8_t *_ucsrb;
        volatile uint8_t *_udr;

        // register magic numbers
        uint8_t         _portEnableBits;        // rx, tx and rx interrupt enables
        uint8_t         _portTxBits;            // tx data and completion interrupt enables
        uint8_t         _u2x;

        // ring buffers
        struct RXBuffer {
                volatile uint8_t head;
                uint8_t         tail;
                uint8_t         bytes[128];     // size must be power of 2 for best results
        };
        struct TXBuffer {
                uint8_t         head;
                volatile uint8_t tail;
                uint8_t         bytes[64];      // size must be power of 2 for best results
        };
        RXBuffer        _rxBuffer;
        TXBuffer        _txBuffer;
};

#endif // FastSerial_h
