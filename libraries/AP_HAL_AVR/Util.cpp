
#include "Util.h"
#include "print_vprintf.h"
using namespace AP_HAL_AVR;

/* Helper class implements AP_HAL::Print so we can use utility/vprintf */
class BufferPrinter : public AP_HAL::Print {
public:
    BufferPrinter(char* str, size_t size)  : _str(str), _size(size), _offs(0) {}
    size_t write(uint8_t c) {
        if (_offs < _size) {
            _str[_offs] = c;
            _offs++;
            return 1;
        } else {
            return 0;
        }
    }
    size_t _offs; 
    char* const  _str;
    const size_t _size;
};

int AVRUtil::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = this->vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int AVRUtil::snprintf_P(char* str, size_t size, const prog_char_t *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = this->vsnprintf_P(str, size, format, ap);
    va_end(ap);
    return res;
}


int AVRUtil::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    BufferPrinter buf(str, size);
    print_vprintf(&buf, 0, format, ap);
    return (int) buf._offs;
}

int AVRUtil::vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap)
{
    BufferPrinter buf(str, size);
    print_vprintf(&buf, 1,(const char*) format, ap);
    return (int) buf._offs;
}


