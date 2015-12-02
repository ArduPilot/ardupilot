#ifndef AP_VEHICLETYPE_H
#define AP_VEHICLETYPE_H

class VehicleType {
public:
    enum vehicle_type {
        VEHICLE_UNKNOWN,
        VEHICLE_COPTER,
        VEHICLE_PLANE,
        VEHICLE_ROVER
    };
};

#endif
