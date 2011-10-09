#ifndef Stream_h
#define Stream_h

#include <inttypes.h>
#include "Print.h"

class Stream : public Print
{
  public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;
};

#endif
