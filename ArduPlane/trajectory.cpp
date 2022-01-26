#include "trajectory.h"


void RalphieTrajectory::init() {
    // TODO: circle
}


void RalphieTrajectory::update() {
    // TODO: wario
}


void RalphieTrajectory::setCurrentWind(Vector3f windEstimate) {

    memcpy(&currentWindEstimate, &windEstimate, sizeof(Vector3f));
}
