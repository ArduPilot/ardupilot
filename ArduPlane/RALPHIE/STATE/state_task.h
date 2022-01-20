#pragma once


#include "../../Plane.h"


typedef struct {

    Vector3f position;
    Vector3f velocity;
    Vector3f angularVelocity;

    float roll;
    float pitch;
    float yaw;

} aircraftState_t;


