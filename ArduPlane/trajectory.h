#ifndef _RALPHIE_TRAJECTORY_H_
#define _RALPHIE_TRAJECTORY_H_

#include "state_task.h"

#define WARIO_TRAJECTORY_SIZE   (100)



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
    void init();

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
