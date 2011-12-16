
#ifndef __ANALOG_SOURCE_H__
#define __ANALOG_SOURCE_H__

class AP_AnalogSource
{
    public:
    virtual float read(void) = 0;
};

#endif // __ANALOG_SOURCE_H__
