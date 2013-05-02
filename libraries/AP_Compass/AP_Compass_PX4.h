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
    static int _mag_fd;
    static Vector3f _sum;
    static uint32_t _count;
    static uint32_t _last_timer;
    static uint64_t _last_timestamp;
    static void _accumulate(void);
    static void _compass_timer(uint32_t now);
};

#endif // AP_Compass_PX4_H

