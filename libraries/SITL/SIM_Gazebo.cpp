/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for ardupilot version of Gazebo
*/

#include "SIM_Gazebo.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;
using namespace std;

namespace SITL {

// Ports for input/output sockets to Gazebo (with IP 127.0.0.1)
#define PORT_DATA_TO_GAZEBO_PLUGIN       9002           // WARNING: previous port was 5002 !
#define PORT_DATA_FROM_GAZEBO_PLUGIN     9003           // WARNING: previous port was 5003 !

#define ROS_PLUGIN_NAME                 "ardupilot_sitl_gazebo_plugin"



Gazebo::Gazebo(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    _is_gazebo_started(false),
    _sock_servos_to_gazebo(true),
    _sock_fdm_from_gazebo(true),
    _is_servos_socket_open(false),
    _is_fdm_socket_open(false),
    _link_status(NOT_INITIALIZED),
    _last_timestamp(0.0),
    _is_sonar_down_present(true),
    _sonar_down(0.0)
{
}

void Gazebo::finalize_creation()
{
  // Only starts ROS/Gazebo from Ardupilot if Ardupilot received the argument asking it
  if (ros_launch_file != NULL) {
      start_ros_gazebo();
  } else {
      _is_gazebo_started = true;
  }
}

/*
  start arducopter_sitl_ros child
 */
void Gazebo::start_ros_gazebo()
{
    // If Gazebo is already started, then there is nothing to do
    if (_is_gazebo_started)
        return;

    pid_t child_pid = fork();
    if (child_pid < 0) {
        fprintf(stderr, "Failed to fork the Ardupilot process to run ROS/Gazebo, err #%d\n", errno);
        return;
    }

    if (child_pid > 0) {
        // We are in the parent process
        printf("ROS fork is now running\n");
        _is_gazebo_started = true;
        return;
    }

    // (child_pid == 0) -> we are in the child process

    close(0);
    open("/dev/null", O_RDONLY);
    // in child
    for (uint8_t i=3; i<100; i++) {
        close(i);
    }

    char terminal_path[300];
    if (autotest_dir != NULL) {
        strcpy(terminal_path, autotest_dir);
        strcat(terminal_path, "/run_in_terminal_window.sh");
        // Note: the 'chdir()' function does not seem to work with 'run_in_terminal_window.sh'
    }

    char roslaunch_cmd[256];
    int ret;

    // Once the ROS  command is completed the new ROS terminal window is left open,
    // so that the user can check on error messages or log.
    // To automatically close the terminal once ROS is completed, append "|| exit 1"
    // at the end of 'roslaunch_cmd'.
    ret = sprintf(roslaunch_cmd, "roslaunch " ROS_PLUGIN_NAME " %s.launch", ros_launch_file);
    if (ret < 0) {
        fprintf(stderr, "Could not format the roslaunch command, check ros-launch size !\n");
        exit(1);
    }

    // Runs ROS/Gazebo in a new terminal to avoid mixing log messages with those from Ardupilot
    ret = execlp(terminal_path,
                 "run_in_terminal_window.sh",
                 "launching ROS",
                 roslaunch_cmd,
                 NULL);

    if (ret != 0) {
        fprintf(stderr, "Failed to run execlp() to launch ROS, err #%d\n", errno);
        _is_gazebo_started = false;
    }

    // Terminates the child process
    exit(1);
}


/*
  open control socket to Gazebo's shim plugin
 */
bool Gazebo::open_servos_socket()
{
    if (_is_servos_socket_open)
        return true;

    if (!_sock_servos_to_gazebo.connect("127.0.0.1", PORT_DATA_TO_GAZEBO_PLUGIN)) {
        fprintf(stderr, "Failed to connect port %d to send control servos to Gazebo\n", PORT_DATA_TO_GAZEBO_PLUGIN);
        return false;
    }
    printf("Opened Gazebo servos send socket\n");
    _sock_servos_to_gazebo.set_blocking(false);
    _is_servos_socket_open = true;

    return true;
}


/*
  open fdm socket from Gazebo's shim plugin
 */
bool Gazebo::open_fdm_socket()
{
    if (_is_fdm_socket_open)
        return true;

    if (!_sock_fdm_from_gazebo.bind("127.0.0.1", PORT_DATA_FROM_GAZEBO_PLUGIN)) {
        fprintf(stderr, "Failed to bind port %d to receive fdm from Gazebo\n", PORT_DATA_TO_GAZEBO_PLUGIN);
        return false;
    }
    printf("Opened Gazebo fdm receive socket\n");
    _sock_fdm_from_gazebo.set_blocking(false);
    _sock_fdm_from_gazebo.reuseaddress();
    _is_fdm_socket_open = true;

    return true;
}


/*
  decode and send servos
*/
void Gazebo::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    int i;

    // Assumes input PWM to range from 1000 to 2000
    for (i=0; i<NB_GAZEBO_SERVOS; i++)
        pkt.motor_speed[i] = ((float)input.servos[i] - 1000.0) / 1000.0f;


    // DEBUG
    static uint16_t prev_servos[16];
    bool servosModif = false;

    for (i=4; i<14; i++) {
        if (input.servos[i] != prev_servos[i]) {
            printf(" servo[%d] modified into %d\n", i, input.servos[i]);
            prev_servos[i] = input.servos[i];
            servosModif = true;
        }
    }
    if (servosModif)
        printf("------------------------\n");
    //DEBUG

    ssize_t sent = _sock_servos_to_gazebo.send(&pkt, sizeof(pkt));

    if (sent < 0) {
        if (errno != EAGAIN) {        // "Try Again" error code
            if (_link_status != ERR_FATAL) {        // Avoids unuseful error message repetitions
                fprintf(stderr, "Fatal: Failed to send on control socket: %s\n", strerror(errno));
                _link_status = ERR_FATAL;
            }
        }
    } else if (sent < (signed)sizeof(pkt)) {
        if (_link_status != ERR_BYTES_LOST) {       // Avoids unuseful error message repetitions
            fprintf(stderr, "Failed to send all bytes on control socket\n");
            _link_status = ERR_BYTES_LOST;
        }
    } else if ((_link_status == ERR_BYTES_LOST) ||
               (_link_status == ERR_BYTES_LOST)) {
        _link_status = RUNNING;                     // This message sending was fine, resets the status
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Gazebo::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (_sock_fdm_from_gazebo.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        send_servos(input);
    }

    // get imu stuff
    accel_body = Vector3f(pkt.imu_linear_acceleration_xyz[0],
                          pkt.imu_linear_acceleration_xyz[1],
                          pkt.imu_linear_acceleration_xyz[2]);

    gyro = Vector3f(pkt.imu_angular_velocity_rpy[0],
                    pkt.imu_angular_velocity_rpy[1],
                    pkt.imu_angular_velocity_rpy[2]);

    // compute dcm from imu orientation
    Quaternion quat(pkt.imu_orientation_quat[0],
                    pkt.imu_orientation_quat[1],
                    pkt.imu_orientation_quat[2],
                    pkt.imu_orientation_quat[3]);
    quat.rotation_matrix(dcm);

    // get velocity NED
    double speedN =  pkt.velocity_xyz[0];
    double speedE =  pkt.velocity_xyz[1];
    double speedD =  pkt.velocity_xyz[2];
    velocity_ef = Vector3f(speedN, speedE, speedD);

    // get position [m] NED
    position = Vector3f(pkt.position_xyz[0],
                        pkt.position_xyz[1],
                        pkt.position_xyz[2]);

    // get GPS position [degrees]->[degrees * 10^7], [m]->[cm]
    location.lat = pkt.position_latlonalt[0] * 1.0e7;
    location.lng = pkt.position_latlonalt[1] * 1.0e7;
    location.alt = pkt.position_latlonalt[2] *1.0e2;

    // Extra sensors -------------------------
    _sonar_down = pkt.sonar_down;
    // ...
    // ---------------------------------------

    // auto-adjust to simulation frame rate
    double deltat = pkt.timestamp - _last_timestamp;    // [seconds]
    time_now_us += deltat * 1.0e6;

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(1.0e6 / deltat);
    }
    _last_timestamp = pkt.timestamp;
}

// From SIM_JSBSim.cpp:
void Gazebo::drain_servos_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = _sock_servos_to_gazebo.recv(buf, buflen, 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(stderr, "error recv on servos socket: %s\n", strerror(errno));
            }
        }
    } while (received > 0);
}


/*
  update the Gazebo simulation by one time step
 */
void Gazebo::update(const struct sitl_input &input)
{
    // If ROS/Gazebo is not running, there is nothing to do
    if (!_is_gazebo_started)
        return;

    // Opens the sockets (if not yet done)
    if (_link_status == NOT_INITIALIZED) {
        if (!open_servos_socket() ||
            !open_fdm_socket()) {
            time_now_us = 1;
            return;
        }
        _link_status = RUNNING;
    }

    uint64_t prev_step_time = time_now_us;

    do {
        send_servos(input);
        recv_fdm(input);

        sync_frame_time();

        drain_servos_socket();

        // Blocks ardupilot until some simulation time has passed
        // Avoids some lingering zero divisions.

    } while (time_now_us == prev_step_time);


    // Usually in every SITL simulation, it is Ardupilot that drives the simulation time.
    // Meaning that if Ardupilot wants to run at 400 Hz (in simulation time), it calls
    // frequently the simulator telling it to advance by 2.5 ms steps.
    //
    // However in Gazebo simulations it is Gazebo that drives the simulation clock,
    // unless a special Gazebo plugin invoking Gazebo's Step(1) method is used.
    //
    // Impact of Gazebo driving the simulation clock:
    //   - Ardupilot cannot pause the simulation. While debugging the code, the UAV will fall down.
    //   - Ardupilot is constantly trying to catch up on the simulation state,
    //     and if Ardupilot takes a bit too long to loop it may miss some steps.
    //   - Gazebo must be defined to run at the the Ardupilot frequency (400 Hz)
    //
    // Impact of Ardupilot driving the simulation clock:
    //   - When Ardupilot's process dies, the simulation dies too. So there is no simulated Pixhawk reboot.
    //   - Ardupilot fixes the simulation step to 400 Hz. It may be difficult to accomodate
    //     multiple UAVs/Robots with different loop frequencies
}

/*
   Redefines SIM_Aircraft's fill_fdm_extras() function, to include additional fields
   covered by ROS/Gazebo (sonars)
*/
void Gazebo::fill_fdm_extras(struct sitl_fdm_extras &fdm_extras) const
{
    // Usual simulators do not support extra sensors. So all 'is_xxx_present' fields
    // are set to false.
    fdm_extras.timestamp_us = time_now_us;
    fdm_extras.sonar_down = _sonar_down;
    fdm_extras.is_sonar_down_present = _is_sonar_down_present;
    fdm_extras.magic = FDM_EXTRAS_MAGIC;
}

} // namespace SITL
