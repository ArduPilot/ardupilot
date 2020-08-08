/*
  low level driver for the TI CC2500 radio on SPI

  With thanks to betaflight
*/

#include "driver_cc2500.h"
#include <utility>

#pragma GCC optimize("O0")

extern const AP_HAL::HAL& hal;

// constructor
Radio_CC2500::Radio_CC2500(AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev) :
    dev(std::move(_dev))
{}

void Radio_CC2500::ReadFifo(uint8_t *dpbuffer, uint8_t len)
{
    (void)dev->read_registers(CC2500_3F_RXFIFO | CC2500_READ_BURST, dpbuffer, len);
}

void Radio_CC2500::WriteFifo(const uint8_t *dpbuffer, uint8_t len)
{
    WriteRegisterMulti(CC2500_3F_TXFIFO | CC2500_WRITE_BURST, dpbuffer, len);
}

void Radio_CC2500::ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t length)
{
    (void)dev->read_registers(address, data, length);
}

void Radio_CC2500::WriteRegisterMulti(uint8_t address, const uint8_t *data, uint8_t length)
{
    uint8_t buf[length+1];
    buf[0] = address;
    memcpy(&buf[1], data, length);
    dev->transfer(buf, length+1, nullptr, 0);
}

uint8_t Radio_CC2500::ReadReg(uint8_t reg)
{
    uint8_t ret = 0;
    (void)dev->read_registers(reg | CC2500_READ_SINGLE, &ret, 1);
    return ret;
}

uint8_t Radio_CC2500::Strobe(uint8_t address)
{
    uint8_t status=0;
    (void)dev->transfer(&address, 1, &status, 1);
    return status;
}

void Radio_CC2500::WriteReg(uint8_t address, uint8_t data)
{
    (void)dev->write_register(address, data);
}

void Radio_CC2500::SetPower(uint8_t power)
{
    const uint8_t patable[8] = {
        0xC6, // -12dbm
        0x97, // -10dbm
        0x6E, // -8dbm
        0x7F, // -6dbm
        0xA9, // -4dbm
        0xBB, // -2dbm
        0xFE, // 0dbm
        0xFF  // 1.5dbm
    };
    if (power > 7) {
        power = 7;
    }
    if (power != last_power) {
        WriteReg(CC2500_3E_PATABLE, patable[power]);
        last_power = power;
    }
}

bool Radio_CC2500::Reset(void)
{
    Strobe(CC2500_SRES);
    hal.scheduler->delay_microseconds(1000);
    // CC2500_SetTxRxMode(TXRX_OFF);
    // RX_EN_off;//off tx
    // TX_EN_off;//off rx
    return ReadReg(CC2500_0E_FREQ1) == 0xC4; // check if reset
}

/*
  write register with up to 5 retries
 */
void Radio_CC2500::WriteRegCheck(uint8_t address, uint8_t data)
{
    uint8_t tries=5;
    while (--tries) {
        dev->write_register(address, data);
        uint8_t v = ReadReg(address);
        if (v == data) {
            break;
        }
    }
}
