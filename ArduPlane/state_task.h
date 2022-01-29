#pragma once

#include <AP_Math/AP_Math.h>


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
     * @brief Aircraft roll  
     * 
     */
    float roll;

    /**
     * @brief Aircraft pitch
     * 
     */
    float pitch;

    /**
     * @brief Aircraft yaw
     * 
     */
    float yaw;

    /**
     * @brief Phase of flight as determined by WARIO algorithm
     * 
     */
    flightPhase_t phase;

} aircraftState_t;


/**
 * @brief Get the Lateral State object
 * 
 * @param state 
 */
void getLateralState(aircraftState_t state);

/**
 * @brief Get the Longitudinal State object
 * 
 * @param state 
 */
void getLongitudinalState(aircraftState_t state);

