/*
  Connection to ArduPilot's SIM_Webots backend, shared by the Webots
  controllers.

  The controller listens on a TCP port; SITL connects to it.  Each simulation
  step the controller sends one line of JSON sensor data and waits for one
  line of JSON controls back, so Webots only advances when SITL does.

  When SITL goes away the controller no longer exits: it waits for the next
  SITL to connect and, when the robot is a supervisor, puts the vehicle back
  where the world started it, so SITL can be restarted without restarting
  Webots.
*/

#ifndef ARDUPILOT_SITL_LINK_H
#define ARDUPILOT_SITL_LINK_H

/* Webots' own bool (a char in C), so these prototypes match the controllers,
   which include the Webots headers first */
#include <webots/types.h>

enum sitl_link_status {
  SITL_LINK_CONTROLS,     /* a control line arrived and was parsed */
  SITL_LINK_NO_CONTROLS,  /* nothing usable yet: re-send the same frame */
  SITL_LINK_CLOSED,       /* SITL disconnected; call sitl_link_accept() */
};

/* parses one control line; returns false if it could not */
typedef bool (*sitl_parse_fn)(const char *line);

/* listen on port; false if the port could not be opened */
bool sitl_link_open(int port);

bool sitl_link_connected(void);

/* block until SITL connects; false on an unrecoverable socket error */
bool sitl_link_accept(void);

/* send one sensor frame and wait (briefly) for SITL's reply */
enum sitl_link_status sitl_link_exchange(const char *frame, sitl_parse_fn parse);

void sitl_link_close(void);

/*
  Remember the robot's pose as the world placed it, and restore it with zero
  velocity.  Both need the robot's "supervisor TRUE"; without it they do
  nothing and the vehicle stays wherever the last SITL session left it.
*/
void vehicle_pose_save(void);
void vehicle_pose_restore(void);

#endif  /* ARDUPILOT_SITL_LINK_H */
