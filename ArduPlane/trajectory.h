#ifndef _RALPHIE_TRAJECTORY_H_
#define _RALPHIE_TRAJECTORY_H_

#include "state_task.h"

#define WARIO_TRAJECTORY_SIZE   (100)

typedef struct {

	float lat;
	float lon;
	float rad;

	float maxAlt;
	float minAlt;

} warioInput_t;


class RalphieTrajectory {

    /**
     * @brief The current estimation of wind direction
     * 
     */
    Vector3f currentWindEstimate;

    /**
     * @brief Array of aircraft states representing the trajectory
     * 
     */
    aircraftState_t waypoints[WARIO_TRAJECTORY_SIZE];

public:

    /**
     * @brief Generate the default circular trajectory 
     * 
     */
    void init(warioInput_t parameters);

    /**
     * @brief Update the trajectory based on the current wind estimate
     * 
     */
    void update();

    /**
     * @brief Set the current wind estimate
     * 
     * @param windEstimate 
     */
    void setCurrentWind(Vector3f windEstimate);

};


#endif /* _RALPHIE_TRAJECTORY_H_ */
