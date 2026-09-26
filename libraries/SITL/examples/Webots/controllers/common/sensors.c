#include <stdio.h>
#include <sys/time.h>
#include "sensors.h"

/*
https://discuss.ardupilot.org/t/copter-x-y-z-which-is-which/6823/2

NED Coordinate System:

The x axis is aligned with the vector to the north pole (tangent to meridians).
The y axis points to the east side (tangent to parallels)
The z axis points to the center of the earth

Body Fixed Frame (Attached to the aircraft)

The x axis points in forward (defined by geometry and not by movement) direction. (= roll axis)
The y axis points to the right (geometrically) (= pitch axis)
The z axis points downwards (geometrically) (= yaw axis)

In order to convert from Body Frame to NED you need to call this function:

copter.ahrs.body_to_earth2D(vel_vector.xy());


The worlds shipped with this controller declare WorldInfo.coordinateSystem "NUE",
so a Webots vector arrives as (North, Up, East).  Every getter below therefore
emits components in the order [0], [2], [1] == (North, East, Up), which is what
SIM_Webots.cpp expects on the wire.
 */

/*
  returns: "roll": 0.0229, "pitch": -0.0005, "yaw": 0.0000
*/
void getInertia(const WbDeviceTag inertialUnit, char *buf)
{
    const double *inertial_directions = wb_inertial_unit_get_roll_pitch_yaw(inertialUnit);

    sprintf(buf, "\"roll\": %f,\"pitch\": %f,\"yaw\": %f",
            inertial_directions[0], inertial_directions[1], inertial_directions[2]);
}

/*
  returns: [23088.669921875, 3876.001220703125, -53204.57421875]
*/
void getCompass(const WbDeviceTag compass, char *buf)
{
    const double *north3D = wb_compass_get_values(compass);

    sprintf(buf, "[%f, %f, %f]", north3D[0], north3D[2], north3D[1]);
}

/*
  returns: "x": 5.5127e-05,"y": -0.00109689,"z": 0.03717955
*/
void getGPS(const WbDeviceTag gps, char *buf)
{
    const double *north3D = wb_gps_get_values(gps);

    sprintf(buf, "\"x\": %f,\"y\": %f,\"z\": %f", north3D[0], north3D[2], north3D[1]);
}

/*
 returns: [0.005074390675872564, 0.22471477091312408, 9.80740737915039]
*/
void getAcc(const WbDeviceTag accelerometer, char *buf)
{
    const double *a = wb_accelerometer_get_values(accelerometer);

    sprintf(buf, "[%f, %f, %f]", a[0], a[2], a[1]);
}


/*
  returns: [-1.0255117643964695e-07, -8.877226775894087e-08, 2.087078510015772e-09]
*/
void getGyro(const WbDeviceTag gyro, char *buf)
{
    const double *g = wb_gyro_get_values(gyro);

    sprintf(buf, "[%f, %f, %f]", g[0], g[2], g[1]);
}


/*
  World-frame velocity, straight from the GPS device.

  This used to be a finite difference of successive GPS positions scaled by
  timestep_scale, guarded by `if (north-delta != 0.0)`.  That guard froze the
  whole vector whenever the north component happened to be bit-identical
  between two steps, and the difference itself was noisy.  wb_gps_get_speed_vector()
  (Webots R2020b and later) reports the true world-frame velocity instead.
*/
void getLinearVelocity(const WbDeviceTag gps, char *buf)
{
    const double *v = wb_gps_get_speed_vector(gps);

    sprintf(buf, "[%f, %f, %f]", v[0], v[2], v[1]);
}


/*
  Pack one sensor frame.  The wire format is line-delimited JSON, one object per
  line, terminated with "\n".

{"ts": 1563544049.284054,
 "vehicle.imu": {"av": [...], "la": [...], "mf": [...]},
 "vehicle.gps": {"x": ..., "y": ..., "z": ...},
 "vehicle.velocity": {"wlv": [...]},
 "vehicle.pose": {"x": ..., "y": ..., "z": ..., "roll": ..., "pitch": ..., "yaw": ...},
 "rpm": [...]}
*/
void getAllSensors(char *buf, WbDeviceTag gyro, WbDeviceTag accelerometer, WbDeviceTag compass,
                   const WbDeviceTag gps, const WbDeviceTag inertial_unit,
                   const char *extra_json)
{
    char compass_buf[150];
    char acc_buf[150];
    char gyro_buf[150];
    char gps_buf[150];
    char inertial_buf[150];
    char linear_velocity_buf[150];

    const double time = wb_robot_get_time(); /* current simulation time in [s] */

    getGyro(gyro, gyro_buf);
    getAcc(accelerometer, acc_buf);
    getCompass(compass, compass_buf);
    getGPS(gps, gps_buf);
    getInertia(inertial_unit, inertial_buf);
    getLinearVelocity(gps, linear_velocity_buf);

    /* note: unlike the original, the sections below are closed properly, so this
       is valid JSON.  SIM_Webots.cpp locates each section by substring, so the
       nesting fix is transparent to it. */
    sprintf(buf,
            "{\"ts\": %lf,"
            "\"vehicle.imu\": {\"av\": %s,\"la\": %s,\"mf\": %s},"
            "\"vehicle.gps\": {%s},"
            "\"vehicle.velocity\": {\"wlv\": %s},"
            "\"vehicle.pose\": {%s,%s}%s}\n",
            time,
            gyro_buf, acc_buf, compass_buf,
            gps_buf,
            linear_velocity_buf,
            gps_buf, inertial_buf,
            extra_json ? extra_json : "");
}
