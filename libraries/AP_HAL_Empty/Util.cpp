
#include "Util.h"
using namespace Empty;

int EmptyUtil::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = this->vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int EmptyUtil::snprintf_P(char* str, size_t size, const prog_char_t *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = this->vsnprintf_P(str, size, format, ap);
    va_end(ap);
    return res;
}


int EmptyUtil::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    return 0;
}

int EmptyUtil::vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap)
{
    return 0;
}


