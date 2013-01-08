/*
 *  AP_HAL_AVR I2C driver. derived from:
 *  I2C.cpp - I2C library
 *  Copyright (c) 2011 Wayne Truchsess.  All right reserved.
 *  Rev 2.0 - September 19th, 2011
 *         - Added support for timeout function to prevent
 *           and recover from bus lockup (thanks to PaulS
 *           and CrossRoads on the Arduino forum)
 *         - Changed return type for stop() from void to
 *           uint8_t to handle timeOut function
 *  Rev 1.0 - August 8th, 2011
 *
 *  This is a modified version of the Arduino Wire/TWI
 *  library.  Functions were rewritten to provide more functionality
 *  and also the use of Repeated Start.  Some I2C devices will not
 *  function correctly without the use of a Repeated Start.  The
 *  initial version of this library only supports the Master.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <AP_HAL.h>
#include "I2CDriver.h"
using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

#ifndef F_CPU
#define CPU_FREQ 16000000L
#else
#define CPU_FREQ F_CPU
#endif

#define START           0x08
#define REPEATED_START  0x10
#define MT_SLA_ACK      0x18
#define MT_DATA_ACK     0x28
#define MR_SLA_ACK      0x40
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define TWI_STATUS      (TWSR & 0xF8)

#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)

#define cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))

void AVRI2CDriver::begin() {
    // activate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    sbi(PORTD, 0);
    sbi(PORTD, 1);

    // initialize twi prescaler and bit rate
    cbi(TWSR, TWPS0);
    cbi(TWSR, TWPS1);
    TWBR = ((CPU_FREQ / 100000) - 16) / 2;
    // enable twi module, acks, and twi interrupt
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);

    // start in high speed. When a driver gets an error it drops it to
    // low speed
    setHighSpeed(true);
}

void AVRI2CDriver::end() {
    TWCR = 0;
}
 
void AVRI2CDriver::setHighSpeed(bool active) {
    if (active) {
        TWBR = ((CPU_FREQ / 400000) - 16) / 2;
    } else {
        TWBR = ((CPU_FREQ / 100000) - 16) / 2;
    }
}

uint8_t AVRI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data){
    uint8_t stat = _start();
    if (stat) goto error;
    stat = _sendAddress(SLA_W(addr));
    if (stat) goto error;
    for (uint8_t i = 0; i < len; i++)
    {
        stat = _sendByte(data[i]);
        if (stat) goto error;
    }
    stat = _stop();
    if (stat) goto error;
    return stat;
error:
    _lockup_count++;
    return stat;
}

uint8_t AVRI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                                    uint8_t len, uint8_t* data){
    uint8_t stat = _start();
    if (stat) goto error;
    stat = _sendAddress(SLA_W(addr));
    if (stat) goto error;
    stat = _sendByte(reg);
    if (stat) goto error;
    for (uint8_t i = 0; i < len; i++)
    {
        stat = _sendByte(data[i]);
        if (stat) goto error;
    }
    stat = _stop();
    if (stat) goto error;
    return stat;
error:
    _lockup_count++;
    return stat;
}

uint8_t AVRI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
        /* Sometimes avr-gcc fails at dereferencing a uint8_t arg. */
        uint8_t data[1];
        data[0] = val;
        return writeRegisters(addr, reg, 1, data);
}

uint8_t AVRI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data){
    uint8_t stat;
    if ( len == 0)
        len = 1;
    uint8_t nackposition = len - 1;
    stat = 0;
    stat = _start();
    if(stat) goto error;
    stat = _sendAddress(SLA_W(addr));
    if(stat) goto error;
    stat = _start();
    if(stat) goto error;
    stat = _sendAddress(SLA_R(addr));
    if(stat) goto error;
    for(uint8_t i = 0; i < len ; i++) {
        if ( i == nackposition ) {
            stat = _receiveByte(false);
            if (stat != MR_DATA_NACK) goto error;
        } else {
            stat = _receiveByte(true);
            if (stat != MR_DATA_ACK) goto error;
        }
        data[i] = TWDR;
    }
    stat = _stop();
    if (stat) goto error;
    return stat;
error:
    _lockup_count++;
    return stat;
}

uint8_t AVRI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                    uint8_t len, uint8_t* data){
    uint8_t stat;
    if ( len == 0)
        len = 1;
    uint8_t nackposition = len - 1;
    stat = 0;
    stat = _start();
    if(stat) goto error;
    stat = _sendAddress(SLA_W(addr));
    if(stat) goto error;
    stat = _sendByte(reg);
    if(stat) goto error;
    stat = _start();
    if(stat) goto error;
    stat = _sendAddress(SLA_R(addr));
    if(stat) goto error;
    for(uint8_t i = 0; i < len ; i++) {
        if ( i == nackposition ) {
            stat = _receiveByte(false);
            if (stat != MR_DATA_NACK) goto error;
        } else {
            stat = _receiveByte(true);
            if (stat != MR_DATA_ACK) goto error;
        }
        data[i] = TWDR;
    }
    stat = _stop();
    if (stat) goto error;
    return stat;
error:
    _lockup_count++;
    return stat;
}

uint8_t AVRI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data) {
    return readRegisters(addr, reg, 1, data);
}

uint8_t AVRI2CDriver::_start() {
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    uint8_t stat = _waitInterrupt();
    if (stat) return stat;

    if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START)) {
        return 0;
    } else {
        return TWI_STATUS;
    }
}

uint8_t AVRI2CDriver::_stop() {
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
    return _waitStop();
}

uint8_t AVRI2CDriver::_sendAddress(uint8_t addr) {
    TWDR = addr;
    TWCR = _BV(TWINT) | _BV(TWEN);
    return _waitInterrupt();
}

uint8_t AVRI2CDriver::_sendByte(uint8_t data) {
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
    uint8_t stat = _waitInterrupt();
    if (stat) return stat;

    if (TWI_STATUS  == MT_DATA_ACK) {
        return 0;
    } else {
        return TWI_STATUS;
    }
}

uint8_t AVRI2CDriver::_receiveByte(bool ack) {
    if (ack) {
        TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
    } else {
        TWCR = _BV(TWINT) | _BV(TWEN);
    }
    uint8_t stat = _waitInterrupt();
    if (stat) return stat;
    return TWI_STATUS;
}

void AVRI2CDriver::_handleLockup() {
    TWCR = 0; /* Releases SDA and SCL lines to high impedance */
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA); /* Reinitialize TWI */
    _lockup_count++;
}

uint8_t AVRI2CDriver::_waitInterrupt() {
    uint32_t start = hal.scheduler->millis();
    if (_timeoutDelay == 0) {
        /* Wait indefinitely for interrupt to go off */
        while (!(TWCR & _BV(TWINT))) { }
    } else {
        /* Wait while polling for timeout */
        while (!(TWCR & _BV(TWINT))) {
            uint32_t current = hal.scheduler->millis();
            if ( current - start >= _timeoutDelay ) {
                _handleLockup();
                return 1;
            }
        }
    }
    return 0;
}

uint8_t AVRI2CDriver::_waitStop() {
    uint32_t start = hal.scheduler->millis();
    if (_timeoutDelay == 0) {
        /* Wait indefinitely for stop condition */
        while( TWCR & _BV(TWSTO) ) { }
    } else  {
        /* Wait while polling for timeout */
        while( TWCR & _BV(TWSTO) ) {
            uint32_t current = hal.scheduler->millis();
            if (current - start >= _timeoutDelay) {
                _handleLockup(); 
                return 1;
            }
        }
    }
    return 0;
}

SIGNAL(TWI_vect)
{
    switch(TWI_STATUS) {
    case 0x20:
    case 0x30:
    case 0x48:
        TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);  // send a stop
        break;
    case 0x38:
    case 0x68:
    case 0x78:
    case 0xB0:
        TWCR = 0;  //releases SDA and SCL lines to high impedance
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);  //reinitialize TWI
        break;
    }
}

#endif
