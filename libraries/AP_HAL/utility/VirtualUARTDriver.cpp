#include "VirtualUARTDriver.h"

#if AP_HAL_VIRTUAL_UART_ENABLED

AP_HAL::VirtualUARTDriver *AP_HAL::VirtualUARTDriver::_instances[MAX_INSTANCES];

AP_HAL::VirtualUARTDriver::VirtualUARTDriver(uint8_t instance)
{
    lock_write_key = 0;
    lock_read_key = 0;
    parity = 0;
    _last_options = 0;
#if AP_UART_MONITOR_ENABLED
    set_monitor_read_buffer(nullptr);
#endif
    if (instance > 0 && instance <= MAX_INSTANCES && _instances[instance - 1] == nullptr) {
        _instance = instance;
        _instances[instance - 1] = this;
    }
}

AP_HAL::VirtualUARTDriver::~VirtualUARTDriver()
{
    if (_instance != 0) {
        _instances[_instance - 1] = nullptr;
    }
    delete _rx;
    delete _tx;
}

AP_HAL::VirtualUARTDriver *AP_HAL::VirtualUARTDriver::get_uart(uint8_t instance)
{
    if (instance == 0 || instance > MAX_INSTANCES) {
        return nullptr;
    }
    return _instances[instance - 1];
}

ByteBuffer *AP_HAL::VirtualUARTDriver::allocate_buffer(uint16_t capacity)
{
    // ByteBuffer reserves one byte to distinguish its empty and full states.
    ByteBuffer *buffer = NEW_NOTHROW ByteBuffer(uint32_t(capacity) + 1);
    if (buffer != nullptr && buffer->get_size() != uint32_t(capacity) + 1) {
        delete buffer;
        return nullptr;
    }
    return buffer;
}

void AP_HAL::VirtualUARTDriver::replace_buffer(ByteBuffer *&current, ByteBuffer *replacement)
{
    if (replacement == nullptr) {
        return;
    }
    if (current != nullptr) {
        ByteBuffer::IoVec vec[2];
        const uint8_t count = current->peekiovec(vec, current->available());
        for (uint8_t i = 0; i < count; i++) {
            replacement->write(vec[i].data, vec[i].len);
        }
        delete current;
    }
    current = replacement;
}

void AP_HAL::VirtualUARTDriver::_begin(uint32_t baud, uint16_t rx_space, uint16_t tx_space)
{
    WITH_SEMAPHORE(_sem);
    if (_instance == 0 || rx_space > MAX_CAPACITY || tx_space > MAX_CAPACITY) {
        return;
    }
    const uint16_t rx_capacity = rx_space < DEFAULT_CAPACITY ? DEFAULT_CAPACITY : rx_space;
    const uint16_t tx_capacity = tx_space < DEFAULT_CAPACITY ? DEFAULT_CAPACITY : tx_space;
    const bool grow_rx = _rx == nullptr || _rx->get_size() <= rx_capacity;
    const bool grow_tx = _tx == nullptr || _tx->get_size() <= tx_capacity;
    ByteBuffer *new_rx = grow_rx ? allocate_buffer(rx_capacity) : nullptr;
    if (grow_rx && new_rx == nullptr) {
        return;
    }
    ByteBuffer *new_tx = grow_tx ? allocate_buffer(tx_capacity) : nullptr;
    if (grow_tx && new_tx == nullptr) {
        delete new_rx;
        return;
    }
    // Allocate both replacements before changing either existing queue.
    replace_buffer(_rx, new_rx);
    replace_buffer(_tx, new_tx);
    if (baud != 0) {
        _baudrate = baud;
    }
}

void AP_HAL::VirtualUARTDriver::_end()
{
    WITH_SEMAPHORE(_sem);
    delete _rx;
    delete _tx;
    _rx = nullptr;
    _tx = nullptr;
    _baudrate = 0;
}

bool AP_HAL::VirtualUARTDriver::is_initialized()
{
    WITH_SEMAPHORE(_sem);
    return _rx != nullptr && _tx != nullptr;
}

bool AP_HAL::VirtualUARTDriver::tx_pending()
{
    WITH_SEMAPHORE(_sem);
    return _tx != nullptr && _tx->available() != 0;
}

uint32_t AP_HAL::VirtualUARTDriver::txspace()
{
    WITH_SEMAPHORE(_sem);
    return _tx != nullptr ? _tx->space() : 0;
}

uint32_t AP_HAL::VirtualUARTDriver::rxspace()
{
    WITH_SEMAPHORE(_sem);
    return _rx != nullptr ? _rx->space() : 0;
}

size_t AP_HAL::VirtualUARTDriver::_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(_sem);
    if (_tx == nullptr || buffer == nullptr) {
        return 0;
    }
    const uint32_t space = _tx->space();
    return _tx->write(buffer, size < space ? size : space);
}

size_t AP_HAL::VirtualUARTDriver::write_rx(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(_sem);
    if (_rx == nullptr || buffer == nullptr) {
        return 0;
    }
    const uint32_t space = _rx->space();
    return _rx->write(buffer, size < space ? size : space);
}

ssize_t AP_HAL::VirtualUARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(_sem);
    if (_rx == nullptr || buffer == nullptr) {
        return 0;
    }
    return _rx->read(buffer, count);
}

size_t AP_HAL::VirtualUARTDriver::read_tx(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(_sem);
    if (_tx == nullptr || buffer == nullptr) {
        return 0;
    }
    return _tx->read(buffer, count);
}

uint32_t AP_HAL::VirtualUARTDriver::_available()
{
    WITH_SEMAPHORE(_sem);
    return _rx != nullptr ? _rx->available() : 0;
}

bool AP_HAL::VirtualUARTDriver::_discard_input()
{
    WITH_SEMAPHORE(_sem);
    if (_rx == nullptr) {
        return false;
    }
    _rx->clear();
    return true;
}

#endif // AP_HAL_VIRTUAL_UART_ENABLED
