
#include "Util.h"
using namespace SMACCM;

int SMACCMUtil::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = this->vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int SMACCMUtil::snprintf_P(char* str, size_t size, const prog_char_t *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = this->vsnprintf_P(str, size, format, ap);
    va_end(ap);
    return res;
}


int SMACCMUtil::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    return 0;
}

int SMACCMUtil::vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap)
{
    return 0;
}


