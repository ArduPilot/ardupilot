#pragma once

#include_next <sys/epoll.h>

#ifndef EPOLLWAKEUP
#define EPOLLWAKEUP (1U<<29)
#endif
