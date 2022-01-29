#include "trajectory.h"


void RalphieTrajectory::init(warioInput_t parameters) {

    // TODO: circle
	//
	

	// (x - xc)^2 + (y - yc)^2 = r^2	
	
	
}


void RalphieTrajectory::update() {
    // TODO: wario
}


void RalphieTrajectory::setCurrentWind(Vector3f windEstimate) {

    memcpy(&currentWindEstimate, &windEstimate, sizeof(Vector3f));
}
