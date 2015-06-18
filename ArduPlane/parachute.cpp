// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"


/* 
   call parachute library update
*/
void Plane::parachute_check()
{
    parachute.update();
}

/*
  parachute_release - trigger the release of the parachute
*/
void Plane::parachute_release()
{
    if (parachute.released()) {
        return;
    }
    
    // send message to gcs and dataflash
    gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("Parachute: Released"));

    // release parachute
    parachute.release();
}

/*
  parachute_manual_release - trigger the release of the parachute,
  after performing some checks for pilot error checks if the vehicle
  is landed
*/
void Plane::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled() || parachute.released()) {
        return;
    }

    // do not release if vehicle is not flying
    if (!is_flying()) {
        // warn user of reason for failure
        gcs_send_text_P(MAV_SEVERITY_WARNING,PSTR("Parachute: not flying"));
        return;
    }

    // if we get this far release parachute
    parachute_release();
}
