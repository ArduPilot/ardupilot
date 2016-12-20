#pragma once

#include_next <termios.h>

#if defined(__ANDROID__) && !defined(HAVE_TERMIOS_CFSETSPEED)

int cfsetspeed(struct termios *s, speed_t  speed)
{
    s->c_cflag = (s->c_cflag & ~CBAUD) | (speed & CBAUD);
    return 0;
}

#endif
