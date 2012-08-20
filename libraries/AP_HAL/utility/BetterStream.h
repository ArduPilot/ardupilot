//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

#ifndef __AP_HAL_UTILITY_BETTERSTREAM_H__
#define __AP_HAL_UTILITY_BETTERSTREAM_H__


class AP_HAL::BetterStream : public AP_HAL::Stream {
public:
    BetterStream(void) {}

    // Stream extensions
    void    print_P(const prog_char_t *);
    void    println_P(const prog_char_t *);
    void    printf(const char *, ...)
                __attribute__ ((format(__printf__, 2, 3)));
    void    _printf_P(const prog_char *, ...)
                __attribute__ ((format(__printf__, 2, 3)));

    virtual int txspace(void);

#define printf_P(fmt, ...) _printf_P((const prog_char *)fmt, ## __VA_ARGS__)

private:
    void    _vprintf(unsigned char, const char *, va_list)
                __attribute__ ((format(__printf__, 3, 0)));


};

#endif // __AP_HAL_UTILITY_BETTERSTREAM_H__

