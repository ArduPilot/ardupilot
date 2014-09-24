/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_Compass_PX4_H
#define AP_Compass_PX4_H

#include "Compass.h"

class AP_Compass_PX4 : public Compass
{
public:
    AP_Compass_PX4() : Compass() {
        product_id = AP_COMPASS_TYPE_PX4;
        _num_instances = 0;
    }
    bool        init(void);
    bool        read(void);
    void        accumulate(void);

    // return the number of compass instances
    uint8_t get_count(void) const { return _num_instances; }

    // return the primary compass
    uint8_t get_primary(void) const;

private:
    uint8_t _num_instances;
    int _mag_fd[COMPASS_MAX_INSTANCES];
    Vector3f _sum[COMPASS_MAX_INSTANCES];
    uint32_t _count[COMPASS_MAX_INSTANCES];
    uint64_t _last_timestamp[COMPASS_MAX_INSTANCES];
};

#endif // AP_Compass_PX4_H

