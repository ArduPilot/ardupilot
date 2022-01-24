#include "state_task.h"

void printSomething() {
    printf("hello world\n");
}

void AircraftState::setPosition(Vector3f positionIn) {

    memcpy(&position, &positionIn, sizeof(Vector3f));
}


void AircraftState::setVelocity(Vector3f velocityIn) {

    memcpy(&velocity, &velocityIn, sizeof(Vector3f));
}


void AircraftState::setAngularVelocity(Vector3f velocityIn) {

    memcpy(&angularVelocity, &velocityIn, sizeof(Vector3f));
}


void AircraftState::setRoll(float rollIn, bool convert) {

    roll = convert ? rollIn : rollIn*DEGREES_TO_RADIANS;
}


void AircraftState::setPitch(float pitchIn, bool convert) {

    pitch = convert ? pitchIn : pitchIn*DEGREES_TO_RADIANS;
}


void AircraftState::setYaw(float yawIn, bool convert) {

    yaw = convert ? yawIn : yawIn*DEGREES_TO_RADIANS;
}







