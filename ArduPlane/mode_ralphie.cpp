
#include "mode.h"
#include "plane.h"


void ModeRalphie::run() {
    /* For control system -> called from Plane::stablize() in Attitude.cpp line 503 */
    printf("RALPHIE CONTROLLING\n");
    return;
}


bool ModeRalphie::_enter() {
    /* Enters the mode, perform tasks that only need to happen on initialization */
    return true;
}


void ModeRalphie::update() {
    /* Called at 400 Hz from scheduler, other miscellaneous items can happen here */
    // printf("IN RALPHIE MODE!!!\n");
    return;
}

void ModeRalphie::navigate() {
    /* For trajectory and navigation -> called from Plane::navigate in ArduPlane.cpp line 109 */
    plane.next_WP_loc.alt = 0;
    printf("RALPHIE NAVIGATING\n");
}