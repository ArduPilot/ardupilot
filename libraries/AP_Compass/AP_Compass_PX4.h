/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_Compass_PX4_H
#define AP_Compass_PX4_H

#include "Compass.h"

class AP_Compass_PX4 : public Compass
{
public:
    AP_Compass_PX4() : Compass() {
        product_id = AP_COMPASS_TYPE_PX4;
    }
    bool        init(void);
    bool        read(void);
    void        accumulate(void);

private:
    int _mag_fd;
    Vector3f _sum;
    uint32_t _count;
};

#endif // AP_Compass_PX4_H

