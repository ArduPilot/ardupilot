#include "UAVCAN_UARTDriver.h"


#define AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_TX            (512*4)
#define AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_RX            (512*4)

extern const AP_HAL::HAL& hal;

UAVCAN_UARTDriver::UAVCAN_UARTDriver()
{
    _write_mutex = hal.util->new_semaphore();
    _read_mutex = hal.util->new_semaphore();
}

/*
  open virtual port
 */
void UAVCAN_UARTDriver::begin(uint32_t baud)
{
    begin(baud, 0, 0);
}

void UAVCAN_UARTDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS)
{
    (void)baud; // unused in virtual ports

    _initialised = false;

    // enforce minimum sizes
    if (rxS < AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_RX) {
        rxS = AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_RX;
    }
    if (txS < AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_TX) {
        txS = AP_UAVCAN_UARTDRIVER_BUF_SIZE_MIN_TX;
    }

    if (_writebuf.set_size(txS) && _readbuf.set_size(rxS)) {
        _initialised = true;
    }
}


/*
  shutdown a Virtual UART
 */
void UAVCAN_UARTDriver::end()
{
    _initialised = false;
    _readbuf.set_size(0);
    _writebuf.set_size(0);
}


/*
  do we have any bytes pending transmission?
 */
bool UAVCAN_UARTDriver::tx_pending()
{
    // any data in SerialManager -> UAVCAN
    return (_writebuf.available() > 0);
}

/*
  return the number of bytes available to be read
 */
uint32_t UAVCAN_UARTDriver::available()
{
    if (!_initialised) {
        return 0;
    }
    // any data in UAVCAN -> SerialManager
    return _readbuf.available();
}

/*
  how many bytes are available in the output buffer?
 */
uint32_t UAVCAN_UARTDriver::txspace()
{
    if (!_initialised) {
        return 0;
    }
    // how much free space available in SerialManager -> UAVCAN
    return _writebuf.space();
}

uint32_t UAVCAN_UARTDriver::handle_inbound(const uint8_t *buffer, uint32_t size)
{
    if (!_read_mutex->take(2)) {
        return 0;
    }

    // load data into UAVCAN -> SerialManager
    uint32_t len = _readbuf.write(buffer, size);
    _read_mutex->give();
    return len;
}

int16_t UAVCAN_UARTDriver::fetch_for_outbound(void)
{
    if (!_initialised) {
        return -1;
    }
    if (!_write_mutex->take(2)) {
        return -2;
    }


    uint8_t byte;
    if (!_writebuf.read_byte(&byte)) {
        _write_mutex->give();
        return -3;
    }

    _write_mutex->give();
    return byte;
}


int16_t UAVCAN_UARTDriver::read()
{
    if (!_initialised) {
        return -1;
    }
    if ( !_read_mutex->take(2)) {
        return -1;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        _read_mutex->give();
        return -1;
    }

    _read_mutex->give();

    return byte;
}

/* implementation of write virtual methods */
size_t UAVCAN_UARTDriver::write(uint8_t c)
{
    if (!_initialised) {
        return 0;
    }
    if (!_write_mutex->take(2)) {
        return 0;
    }

    uint32_t len = _writebuf.write(&c, 1);
    _write_mutex->give();

    return len;
}

/*
  write size bytes to the write buffer
 */
size_t UAVCAN_UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
        return 0;
    }
    if (!_write_mutex->take(2)) {
        return 0;
    }

    uint32_t len = _writebuf.write(buffer, size);
    _write_mutex->give();

    return len;
}

