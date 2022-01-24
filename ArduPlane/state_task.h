#ifndef _RALPHIE_STATE_TASK_H_
#define _RALPHIE_STATE_TASK_H_

#include "Plane.h"

#define DEGREES_TO_RADIANS  (3.14/180) /* TODO: Definitely a conversion for this already */

void printSomething();

/**
 * @brief Enumeration with values representing the current phase of flight as indicated
 * by the WARIO trajectory algorithm.
 * 
 */
typedef enum {

    /**
     * @brief Default trajectory pattern, circular
     * 
     */
    FLIGHT_PHASE_CIRCLE,

    /**
     * @brief Straight portion of squircle 
     * 
     */
    FLIGHT_PHASE_STRAIGHT,

    /**
     * @brief Curved portion of squircle
     * 
     */
    FLIGHT_PHASE_SEMI_CIRCLE,

    /**
     * @brief Transition path between squircles
     * 
     */
    FLIGHT_PHASE_TRANSITION
} flightPhase_t;


/**
 * @brief A class to represent an aircraft's full state. Contains methods to get and 
 * set all relevant vectors and angles. For the purposes of the LQT controller, contains
 * methods to return 6x1 vectors of the lateral and longitudinal states. Additionally,
 * contains the phase WARIO trajectory the state represents.
 * 
 */
class AircraftState {

    /**
     * @brief Inertial position (x, y, z)
     * 
     */
    Vector3f position;

    /**
     * @brief Inertial velocity (u, v, w)
     * 
     */
    Vector3f velocity;

    /**
     * @brief Angular velocity (p, q, r)
     * 
     */
    Vector3f angularVelocity;

    /**
     * @brief Roll angle in radians (phi)
     * 
     */
    float roll;

    /**
     * @brief Pitch angle in radians (theta)
     * 
     */
    float pitch;

    /**
     * @brief Yaw angle in radians (psi)
     * 
     */
    float yaw;

    flightPhase_t flightPhase;

public:

    /**
     * @brief Set the inertial position 
     * 
     * @param positionIn <Vector3f>
     */
    void setPosition(Vector3f positionIn);

    /**
     * @brief Set the inertial velocity
     * 
     * @param velocityIn <Vector3f>
     */
    void setVelocity(Vector3f velocityIn);

    /**
     * @brief Set the angular velocity 
     * 
     * @param velocityIn <Vector3f>
     */
    void setAngularVelocity(Vector3f velocityIn);

    /**
     * @brief Set the roll angle
     * 
     * @param rollIn <float>
     * @param convert <bool> default = false
     */
    void setRoll(float rollIn, bool convert);

    /**
     * @brief Set the pitch angle
     * 
     * @param pitchIn <float>
     * @param convert <bool> default = false
     */
    void setPitch(float pitchIn, bool convert);

    /**
     * @brief Set the yaw angle
     * 
     * @param yawIn <float>
     * @param convert <bool> default = false
     */
    void setYaw(float yawIn, bool convert);

    /**
     * @brief Get the inertial position
     * 
     * @param position <Vector3f>
     */
    void getPosition(Vector3f &position);

    /**
     * @brief Get the inertial velocity
     * 
     * @param velocity <Vector3f>
     */
    void getVelocity(Vector3f &velocity);

    /**
     * @brief Get the angular velocity
     * 
     * @param angularVelocity <Vector3f>
     */
    void getAngularVelocity(Vector3f &angularVelocity);

    /**
     * @brief Get the roll angle
     * 
     * @return float 
     */
    float getRoll();

    /**
     * @brief Get the pitch angle
     * 
     * @return float 
     */
    float getPitch(); 

    /**
     * @brief Get the yaw angle
     * 
     * @return float 
     */
    float getYaw();

    /**
     * @brief Set the flight phase
     * 
     * @param phaseIn <flightPhase_t>
     */
    void setPhase(flightPhase_t phaseIn);

    /**
     * @brief Get the flight phase
     * 
     * @return flightPhase_t 
     */
    flightPhase_t phase();
};






/**
 * @brief 
 * 
 */
typedef struct {

    /**
     * @brief Inertial position (x, y, z)
     * 
     */
    Vector3f position;

    /**
     * @brief Inertial velocity (u, v, w)
     * 
     */
    Vector3f velocity;

    /**
     * @brief Angular velocity (p, w, r)
     * 
     */
    Vector3f angularVelocity;

    /**
     * @brief Auler angles (phi, theta, psi)
     * 
     */
    float roll;
    float pitch;
    float yaw;

    /**
     * @brief Phase of flight as determined by WARIO algorithm
     * 
     */
    flightPhase_t phase;

} aircraftState_t;


#endif /* _RALPHIE_STATE_TASK_H_ */
