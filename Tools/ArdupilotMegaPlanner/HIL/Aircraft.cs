using System;
using System.Collections.Generic;
using System.Text;
using YLScsDrawing.Drawing3d;
using ArdupilotMega.HIL;

namespace ArdupilotMega.HIL
{
    public class Aircraft : Utils
    {
        Aircraft self;
        public double home_latitude = 0;
        public double home_longitude = 0;
        public double home_altitude = 0;
        public double ground_level = 0;
        public double frame_height = 0.0;

        public double latitude = 0;
        public double longitude = 0;
        public double altitude = 0;

        public Matrix3 dcm = new Matrix3();

        public Vector3 gyro = new Vector3(0, 0, 0);

        public Vector3 accel_body = new Vector3(0, 0, 0);

        public Vector3 velocity = new Vector3(0, 0, 0); //# m/s, North, East, Up
        public Vector3 position = new Vector3(0, 0, 0); //# m North, East, Up
        public double mass = 0.0;
        public double update_frequency = 50;//# in Hz
        public double gravity = 9.80665;//# m/s/s
        public Vector3 accelerometer = new Vector3(0, 0, -9.80665);

        public double roll = 0;
        public double pitch = 0; 
        public double yaw = 0;

        public Wind wind = new Wind("0,0,0");

        public Aircraft()
        {
            self = this;
        }

        public bool on_ground(Vector3 position = null)
        {
            // '''return true if we are on the ground'''
            if (position == null)
                position = self.position;
            return (-position.z) + self.home_altitude <= self.ground_level + self.frame_height;
        }

        public void update_position(double delta_time = 0)
        {
            //'''update lat/lon/alt from position'''

            double bearing = degrees(atan2(self.position.y, self.position.x));
            double distance = sqrt(self.position.x * self.position.x + self.position.y * self.position.y);

            gps_newpos(self.home_latitude, self.home_longitude,
                                                              bearing, distance);

            self.altitude = self.home_altitude - self.position.z;

            Vector3 velocity_body = self.dcm.transposed() * self.velocity;

            self.accelerometer = self.accel_body.copy();

        }

        public void gps_newpos(double lat, double lon, double bearing, double distance)
        {
            // '''extrapolate latitude/longitude given a heading and distance 
            //   thanks to http://www.movable-type.co.uk/scripts/latlong.html
            //  '''
            // from math import sin, asin, cos, atan2, radians, degrees
            double radius_of_earth = 6378100.0;//# in meters

            double lat1 = radians(lat);
            double lon1 = radians(lon);
            double brng = radians(bearing);
            double dr = distance / radius_of_earth;

            double lat2 = asin(sin(lat1) * cos(dr) +
                        cos(lat1) * sin(dr) * cos(brng));
            double lon2 = lon1 + atan2(sin(brng) * sin(dr) * cos(lat1),
                                cos(dr) - sin(lat1) * sin(lat2));

            latitude = degrees(lat2);
            longitude = degrees(lon2);
            //return (degrees(lat2), degrees(lon2));
        }

    }
}