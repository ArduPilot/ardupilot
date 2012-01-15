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

        public double pitch = 0.0;     //# degrees
        public double roll = 0.0;    //# degrees
        public double yaw = 0.0;   //# degrees

        public double pitch_rate = 0.0;  //# degrees/s
        public double roll_rate = 0.0;  //# degrees/s
        public double yaw_rate = 0.0;  //# degrees/s

        public double pDeg = 0.0;   // degrees/s
        public double qDeg = 0.0;   // degrees/s
        public double rDeg = 0.0;   // degrees/s

        public Vector3d velocity = new Vector3d(0, 0, 0); //# m/s, North, East, Up
        public Vector3d position = new Vector3d(0, 0, 0); //# m North, East, Up
        public Vector3d accel = new Vector3d(0, 0, 0); //# m/s/s North, East, Up
        public double mass = 0.0;
        public double update_frequency = 50;//# in Hz
        public double gravity = 9.8;//# m/s/s
        public Vector3d accelerometer = new Vector3d(0, 0, -9.8);

        public Aircraft()
        {
            self = this;
        }

        public void normalise()
        {
            roll = norm(roll, -180, 180);
            pitch = norm(pitch, -180, 180);
            yaw = norm(yaw, 0, 360);
        }

        double norm(double angle, double min, double max)
        {
            while (angle > max)
                angle -= 360;
            while (angle < min)
                angle += 360;
            return angle;
        }

        public void update_position()
        {
            //'''update lat/lon/alt from position'''

            double radius_of_earth = 6378100.0;// # in meters
            double dlat = rad2deg * (Math.Atan(position.X / radius_of_earth));
            double dlon = rad2deg * (Math.Atan(position.Y / radius_of_earth));

            altitude = home_altitude + position.Z;
            latitude = home_latitude + dlat;
            longitude = home_longitude + dlon;

            // work out what the accelerometer would see
            double xAccel = sin(radians(self.pitch)) * cos(radians(self.roll));
            double yAccel = -sin(radians(self.roll)) * cos(radians(self.pitch));
            double zAccel = -cos(radians(self.roll)) * cos(radians(self.pitch));
            double scale = 9.81 / sqrt((xAccel * xAccel) + (yAccel * yAccel) + (zAccel * zAccel));
            xAccel *= scale;
            yAccel *= scale;
            zAccel *= scale;
            self.accelerometer = new Vector3d(xAccel, yAccel, zAccel);

        }



    }
}