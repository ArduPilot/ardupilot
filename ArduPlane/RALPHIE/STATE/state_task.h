
#include "../../Plane.h"

#define WARIO_TRAJECTORY_SIZE   (100)

typedef struct {

    Vector3f position;
    Vector3f velocity;
    Vector3f angularVelocity;

    float roll;
    float pitch;
    float yaw;

} aircraftState_t;


