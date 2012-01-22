using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using YLScsDrawing.Drawing3d;
using ArdupilotMega.HIL;

namespace ArdupilotMega.HIL
{
    public class Motor : Utils
    {
        const bool True = true;
        const bool False = false;

        public Motor self;
        public double angle;
        public bool clockwise;
        public double servo;

        public Motor(double angle, bool clockwise, double servo)
        {
            self = this;
            self.angle = angle;
            self.clockwise = clockwise;
            self.servo = servo;
        }

        public static Motor[] build_motors(string frame)
        {
            Motor[] motors = new HIL.Motor[8];
            frame = frame.ToLower();
            if (frame.Contains("quad") || frame.Contains("quadx"))
            {
                motors = new HIL.Motor[] {
            new Motor(90,  False,  1),
            new Motor(270, False,  2),
            new Motor(0,   True,   3),
            new Motor(180, True,   4),
                };
                if (frame.Contains("quadx"))
                {
                    foreach (int i in range(4))
                        motors[i].angle -= 45.0;
                }
            }

            else if (frame.Contains("y6"))
            {
                motors = new HIL.Motor[] {
            new Motor(60,   False, 1),
            new Motor(60,   True,  7),
            new Motor(180,  True,  4),
            new Motor(180,  False, 8),
            new Motor(-60,  True,  2),
            new Motor(-60,  False, 3),
            };
            }
            else if (frame.Contains("hexa") || frame.Contains("hexax"))
            {
                motors = new HIL.Motor[] {
            new Motor(0,   True,  1),
            new Motor(60,  False, 4),
            new Motor(120, True,  8),
            new Motor(180, False, 2),
            new Motor(240, True,  3),
            new Motor(300, False, 7),
           };
            }
            else if (frame.Contains("hexax"))
            {
                motors = new HIL.Motor[] {
            new Motor(30,  False,  7),
            new Motor(90,  True,   1),
            new Motor(150, False,  4),
            new Motor(210, True,   8),
            new Motor(270, False,  2),
            new Motor(330, True,   3),
           };
            }
            else if (frame.Contains("octa") || frame.Contains("octax"))
            {
                motors = new HIL.Motor[] {
            new Motor(0,    True,  1),
            new Motor(180,  True,  2),
            new Motor(45,   False, 3),
            new Motor(135,  False, 4),
            new Motor(-45,  False, 7),
            new Motor(-135, False, 8),
            new Motor(270,  True, 10),
            new Motor(90,   True, 11),
        };
                if (frame.Contains("octax"))
                {
                    foreach (int i in range(8))
                        motors[i].angle += 22.5;
                }
            }
            return motors;
        }
    }

    public class QuadCopter : Aircraft
    {
        QuadCopter self;

        int framecount = 0;
        DateTime seconds = DateTime.Now;

        double[] motor_speed = null;

        double hover_throttle;
        double terminal_velocity;
        double terminal_rotation_rate;
        Motor[] motors;

        Vector3d old_position;


        //# scaling from total motor power to Newtons. Allows the copter
        //# to hover against gravity when each motor is at hover_throttle
        double thrust_scale;

        DateTime last_time;

        public QuadCopter(string frame = "quad")
        {
            self = this;

            motors = Motor.build_motors(frame);

            mass = 1.0;// # Kg
            frame_height = 0.1;
            motor_speed = new double[motors.Length];
            hover_throttle = 0.37;
            terminal_velocity = 30.0;
            terminal_rotation_rate = 4 * 360.0;

            thrust_scale = (mass * gravity) / (motors.Length * hover_throttle);

            last_time = DateTime.Now;
        }

        double scale_rc(int sn, float servo, float min, float max)
        {
            float min_pwm = 1000;
            float max_pwm = 2000;
            //'''scale a PWM value'''       # default to servo range of 1000 to 2000  
            if (MainV2.comPort.param.Count > 0)
            {
                min_pwm = int.Parse(MainV2.comPort.param["RC3_MIN"].ToString());
                max_pwm = int.Parse(MainV2.comPort.param["RC3_MAX"].ToString());
            }

            float p = (servo - min_pwm) / (max_pwm - min_pwm);

            float v = min + p * (max - min);

            if (v < min)
                v = min;
            if (v > max)
                v = max;
            return v;
        }

        public void update(ref double[] servos, ArdupilotMega.GCSViews.Simulation.FGNetFDM fdm)
        {
            for (int i = 0; i < servos.Length; i++)
            {
                if (servos[i] <= 0.0)
                {
                    motor_speed[i] = 0;
                }
                else
                {
                    motor_speed[i] = scale_rc(i, (float)servos[i], 0.0f, 1.0f);
                    //servos[i] = motor_speed[i];
                }
            }
            double[] m = motor_speed;

            /*
            roll = 0;
            pitch = 0;
            yaw = 0;
            roll_rate = 0;
            pitch_rate = 0;
            yaw_rate = 0;
            */

            //            Console.WriteLine("\nin m {0:0.000000} {1:0.000000} {2:0.000000} {3:0.000000}", m[0], m[1], m[2], m[3]);
            //            Console.WriteLine("in vel {0:0.000000} {1:0.000000} {2:0.000000}", velocity.X, velocity.Y, velocity.Z);
            //Console.WriteLine("in r {0:0.000000} p {1:0.000000} y {2:0.000000}    - r {3:0.000000} p {4:0.000000} y {5:0.000000}", roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate);


            //            m[0] *= 0.5;

            //# how much time has passed?
            DateTime t = DateTime.Now;
            TimeSpan delta_time = t - last_time; // 0.02
            last_time = t;

            if (delta_time.TotalMilliseconds > 100) // somethings wrong / debug
            {
                delta_time = new TimeSpan(0, 0, 0, 0, 20);
            }

             // rotational acceleration, in degrees/s/s, in body frame
            double roll_accel = 0.0;
            double pitch_accel = 0.0;
            double yaw_accel = 0.0;
            double thrust = 0.0;

            foreach (var i in range((self.motors.Length)))
            {
                roll_accel += -5000.0 * sin(radians(self.motors[i].angle)) * m[i];
                pitch_accel += 5000.0 * cos(radians(self.motors[i].angle)) * m[i];
                if (self.motors[i].clockwise)
                {
                    yaw_accel -= m[i] * 400.0;
                }
                else
                {
                    yaw_accel += m[i] * 400.0;
                }
                thrust += m[i] * self.thrust_scale; // newtons
            }

        // rotational resistance
            roll_accel -= (self.pDeg / self.terminal_rotation_rate) * 5000.0;
            pitch_accel -= (self.qDeg / self.terminal_rotation_rate) * 5000.0;
            yaw_accel -= (self.rDeg / self.terminal_rotation_rate) * 400.0;

            //Console.WriteLine("roll {0} {1} {2}", roll_accel, roll_rate, roll);

            //# update rotational rates in body frame
        self.pDeg  += roll_accel * delta_time.TotalSeconds;
        self.qDeg  += pitch_accel * delta_time.TotalSeconds;
        self.rDeg += yaw_accel * delta_time.TotalSeconds;

            // Console.WriteLine("roll {0} {1} {2}", roll_accel, roll_rate, roll);

                    // calculate rates in earth frame
            
             var answer =  BodyRatesToEarthRates(self.roll, self.pitch, self.yaw,
                                                      self.pDeg, self.qDeg, self.rDeg);
                    self.roll_rate = answer.Item1;
         self.pitch_rate = answer.Item2;
         self.yaw_rate = answer.Item3;
            
         //self.roll_rate = pDeg;
         //self.pitch_rate = qDeg;
         //self.yaw_rate = rDeg;

            //# update rotation
            roll += roll_rate * delta_time.TotalSeconds;
            pitch += pitch_rate * delta_time.TotalSeconds;
            yaw += yaw_rate * delta_time.TotalSeconds;

            //            Console.WriteLine("roll {0} {1} {2}", roll_accel, roll_rate, roll);

            //Console.WriteLine("r {0:0.0} p {1:0.0} y {2:0.0}    - r {3:0.0} p {4:0.0} y {5:0.0}  ms {6:0.000}", roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, delta_time.TotalSeconds);

            //# air resistance
            Vector3d air_resistance = -velocity * (gravity / terminal_velocity);

            //# normalise rotations
            normalise();

            double accel = thrust / mass;

            //Console.WriteLine("in {0:0.000000} {1:0.000000} {2:0.000000} {3:0.000000}", roll, pitch, yaw, accel);

            Vector3d accel3D = RPY_to_XYZ(roll, pitch, yaw, accel);
            //Console.WriteLine("accel3D " + accel3D.X + " " + accel3D.Y + " " + accel3D.Z);
            accel3D += new Vector3d(0, 0, -gravity);
            accel3D += air_resistance;

            Random rand = new Random();

            //velocity.X += .02 + rand.NextDouble() * .03;
            //velocity.Y += .02 + rand.NextDouble() * .03;

            //# new velocity vector
            velocity += accel3D * delta_time.TotalSeconds;
            this.accel = accel3D;

            //# new position vector
            old_position = new Vector3d(position);
            position += velocity * delta_time.TotalSeconds;

            //            Console.WriteLine(fdm.agl + " "+ fdm.altitude);

            //Console.WriteLine("Z {0} halt {1}  < gl {2} fh {3}" ,position.Z , home_altitude , ground_level , frame_height);

            if (home_latitude == 0 || home_latitude > 90 || home_latitude < -90 || home_longitude == 0)
            {
                this.home_latitude = fdm.latitude * rad2deg;
                this.home_longitude = fdm.longitude * rad2deg;
                this.home_altitude = fdm.altitude * ft2m;
                this.ground_level = this.home_altitude;

                this.altitude = fdm.altitude * ft2m;
                this.latitude = fdm.latitude * rad2deg;
                this.longitude = fdm.longitude * rad2deg;
            }

            //# constrain height to the ground
            if (position.Z + home_altitude < ground_level + frame_height)
            {
                if (old_position.Z + home_altitude > ground_level + frame_height)
                {
                    //                    Console.WriteLine("Hit ground at {0} m/s", (-velocity.Z));
                }
                velocity = new Vector3d(0, 0, 0);
                roll_rate = 0;
                pitch_rate = 0;
                yaw_rate = 0;
                roll = 0;
                pitch = 0;
                position = new Vector3d(position.X, position.Y,
                                               ground_level + frame_height - home_altitude + 0);
                // Console.WriteLine("here " + position.Z);
            }

            //# update lat/lon/altitude
            update_position();

            // send to apm
#if MAVLINK10
            ArdupilotMega.MAVLink.__mavlink_gps_raw_int_t gps = new ArdupilotMega.MAVLink.__mavlink_gps_raw_int_t();
#else
            ArdupilotMega.MAVLink.__mavlink_gps_raw_t gps = new ArdupilotMega.MAVLink.__mavlink_gps_raw_t();
#endif

            ArdupilotMega.MAVLink.__mavlink_attitude_t att = new ArdupilotMega.MAVLink.__mavlink_attitude_t();

            ArdupilotMega.MAVLink.__mavlink_vfr_hud_t asp = new ArdupilotMega.MAVLink.__mavlink_vfr_hud_t();

            att.roll = (float)roll * deg2rad;
            att.pitch = (float)pitch * deg2rad;
            att.yaw = (float)yaw * deg2rad;
            att.rollspeed = (float)roll_rate *deg2rad;
            att.pitchspeed = (float)pitch_rate *deg2rad;
            att.yawspeed = (float)yaw_rate *deg2rad;

#if MAVLINK10

            gps.alt = ((int)(altitude * 1000));
            gps.fix_type = 3;
            gps.vel = (ushort)(Math.Sqrt((velocity.X * velocity.X) + (velocity.Y * velocity.Y)) * 100);
            gps.cog = (ushort)((((Math.Atan2(velocity.Y, velocity.X) * rad2deg) + 360) % 360) * 100);
            gps.lat = (int)(latitude* 1.0e7);
            gps.lon = (int)(longitude * 1.0e7);
            gps.time_usec = ((ulong)DateTime.Now.Ticks);

            asp.airspeed = gps.vel;
			
#else
            gps.alt = ((float)(altitude));
            gps.fix_type = 3;

            gps.lat = ((float)latitude);
            gps.lon = ((float)longitude);
            gps.usec = ((ulong)DateTime.Now.Ticks);

            //Random rand = new Random();
            //gps.alt += (rand.Next(100) - 50) / 100.0f;
            //gps.lat += (float)((rand.NextDouble() - 0.5) * 0.00005);
            //gps.lon += (float)((rand.NextDouble() - 0.5) * 0.00005);
            //gps.v += (float)(rand.NextDouble() - 0.5) * 1.0f;

            gps.v = ((float)Math.Sqrt((velocity.X * velocity.X) + (velocity.Y * velocity.Y)));
            gps.hdg = (float)(((Math.Atan2(velocity.Y, velocity.X) * rad2deg) + 360) % 360); ;

            asp.airspeed = gps.v;
#endif

            MainV2.comPort.sendPacket(att);

            MAVLink.__mavlink_raw_pressure_t pres = new MAVLink.__mavlink_raw_pressure_t();
            double calc = (101325 * Math.Pow(1 - 2.25577 * Math.Pow(10, -5) * gps.alt, 5.25588));
            pres.press_diff1 = (short)(int)(calc); // 0 alt is 0 pa

            MainV2.comPort.sendPacket(pres);

            MainV2.comPort.sendPacket(asp);

            if (framecount % 120 == 0)
            {// 50 / 10 = 5 hz
                MainV2.comPort.sendPacket(gps);
                //Console.WriteLine(DateTime.Now.Millisecond + " GPS" );
            }

            framecount++;
        }

        public static Vector3d RPY_to_XYZ(double roll, double pitch, double yaw, double length)
        {
            Vector3d v = new Vector3d(0, 0, length);
            v = new_rotate_euler(-deg2rad * (pitch), 0, -deg2rad * (roll)) * v;
            v = new_rotate_euler(0, deg2rad * (yaw), 0) * v;
            return v;
        }

        public static Quaternion new_rotate_euler(double heading, double attitude, double bank)
        {
            Quaternion Q = new Quaternion();
            double c1 = Math.Cos(heading / 2);
            double s1 = Math.Sin(heading / 2);
            double c2 = Math.Cos(attitude / 2);
            double s2 = Math.Sin(attitude / 2);
            double c3 = Math.Cos(bank / 2);
            double s3 = Math.Sin(bank / 2);

            Q.W = c1 * c2 * c3 - s1 * s2 * s3;
            Q.X = s1 * s2 * c3 + c1 * c2 * s3;
            Q.Y = s1 * c2 * c3 + c1 * s2 * s3;
            Q.Z = c1 * s2 * c3 - s1 * c2 * s3;
            return Q;
        }
    }
}