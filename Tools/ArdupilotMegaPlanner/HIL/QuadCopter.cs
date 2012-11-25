using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using log4net;
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
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        QuadCopter self;

        DateTime seconds = DateTime.Now;

        double[] motor_speed = null;

        double hover_throttle;
        double terminal_velocity;
        double terminal_rotation_rate;
        Motor[] motors;

        Vector3 old_position;


        //# scaling from total motor power to Newtons. Allows the copter
        //# to hover against gravity when each motor is at hover_throttle
        double thrust_scale;

        DateTime last_time;

        public QuadCopter(string frame = "quad")
        {
            self = this;

            motors = Motor.build_motors(frame);
            motor_speed = new double[motors.Length];
            mass = 1.0;// # Kg
            frame_height = 0.1;
            
            hover_throttle = 0.37;
            terminal_velocity = 30.0;
            terminal_rotation_rate = 4 * (360.0 * deg2rad);

            thrust_scale = (mass * gravity) / (motors.Length * hover_throttle);

            last_time = DateTime.Now;
        }

        double scale_rc(int sn, float servo, float min, float max)
        {
            return ((servo - 1000) / 1000.0);
        }



        public void update(ref double[] servos, ArdupilotMega.GCSViews.Simulation.FGNetFDM fdm)
        {
            for (int i = 0; i < servos.Length; i++)
            {
                var servo = servos[(int)self.motors[i].servo - 1];
                if (servo <= 0.0)
                {
                    motor_speed[i] = 0;
                }
                else
                {
                    motor_speed[i] = scale_rc(i, (float)servo, 0.0f, 1.0f);
                    //servos[i] = motor_speed[i];
                }
            }
            double[] m = motor_speed;

            //# how much time has passed?
            DateTime t = DateTime.Now;
            TimeSpan delta_time = t - last_time; // 0.02
            last_time = t;

            if (delta_time.TotalMilliseconds > 100) // somethings wrong / debug
            {
                delta_time = new TimeSpan(0, 0, 0, 0, 20);
            }

            // rotational acceleration, in degrees/s/s, in body frame
            Vector3 rot_accel = new Vector3(0, 0, 0);
            double thrust = 0.0;

            foreach (var i in range((self.motors.Length)))
            {
                rot_accel.x += -radians(5000.0) * sin(radians(self.motors[i].angle)) * m[i];
                rot_accel.y += radians(5000.0) * cos(radians(self.motors[i].angle)) * m[i];
                if (self.motors[i].clockwise)
                {
                    rot_accel.z -= m[i] * radians(400.0);
                }
                else
                {
                    rot_accel.z += m[i] * radians(400.0);
                }
                thrust += m[i] * self.thrust_scale; // newtons
            }

            // rotational air resistance
            rot_accel.x -= self.gyro.x * radians(5000.0) / self.terminal_rotation_rate;
            rot_accel.y -= self.gyro.y * radians(5000.0) / self.terminal_rotation_rate;
            rot_accel.z -= self.gyro.z * radians(400.0) / self.terminal_rotation_rate;

          //  Console.WriteLine("rot_accel " + rot_accel.ToString());

            // update rotational rates in body frame
            self.gyro += rot_accel * delta_time.TotalSeconds;

         //   Console.WriteLine("gyro " + gyro.ToString());

            // update attitude
            self.dcm.rotate(self.gyro * delta_time.TotalSeconds);
            self.dcm.normalize();


            // air resistance
            Vector3 air_resistance = -self.velocity * (self.gravity / self.terminal_velocity);

            accel_body = new Vector3(0, 0, -thrust / self.mass);
            Vector3 accel_earth = self.dcm * accel_body;
            accel_earth += new Vector3(0, 0, self.gravity);
            accel_earth += air_resistance;

            // add in some wind (turn force into accel by dividing by mass).
            accel_earth += self.wind.drag(self.velocity) / self.mass;

            // if we're on the ground, then our vertical acceleration is limited
            // to zero. This effectively adds the force of the ground on the aircraft
            if (self.on_ground() && accel_earth.z > 0)
                accel_earth.z = 0;

            // work out acceleration as seen by the accelerometers. It sees the kinematic
            // acceleration (ie. real movement), plus gravity
            self.accel_body = self.dcm.transposed() * (accel_earth + new Vector3(0, 0, -self.gravity));

            // new velocity vector
            self.velocity += accel_earth * delta_time.TotalSeconds;

            if (double.IsNaN(velocity.x) || double.IsNaN(velocity.y) || double.IsNaN(velocity.z))
                velocity = new Vector3();

            // new position vector
            old_position = self.position.copy();
            self.position += self.velocity * delta_time.TotalSeconds;

            if (home_latitude == 0)
            {
                home_latitude = fdm.latitude * rad2deg;
                home_longitude = fdm.longitude * rad2deg;
                home_altitude = altitude;
            }

            // constrain height to the ground
            if (self.on_ground())
            {
                if (!self.on_ground(old_position))
                    Console.WriteLine("Hit ground at {0} m/s", (self.velocity.z));

                self.velocity = new Vector3(0, 0, 0);
                // zero roll/pitch, but keep yaw
                double r = 0;
                double p = 0;
                double y = 0;
                self.dcm.to_euler(ref r, ref p, ref y);
                self.dcm.from_euler(0, 0, y);

                self.position = new Vector3(self.position.x, self.position.y,
                                        -(self.ground_level + self.frame_height - self.home_altitude));
            }

            // update lat/lon/altitude
            self.update_position(delta_time.TotalSeconds);

            // send to apm
            MAVLink.mavlink_hil_state_t hilstate = new MAVLink.mavlink_hil_state_t();

            hilstate.time_usec = (UInt64)DateTime.Now.Ticks; // microsec

            hilstate.lat = (int)(latitude * 1e7); // * 1E7
            hilstate.lon = (int)(longitude * 1e7); // * 1E7
            hilstate.alt = (int)(altitude * 1000); // mm

            self.dcm.to_euler(ref roll, ref pitch, ref yaw);

            if (double.IsNaN(roll))
            {
                self.dcm.identity();
            }

            hilstate.roll = (float)roll;
            hilstate.pitch = (float)pitch;
            hilstate.yaw = (float)yaw;

            Vector3 earth_rates = Utils.BodyRatesToEarthRates(dcm, gyro);

            hilstate.rollspeed = (float)earth_rates.x;
            hilstate.pitchspeed = (float)earth_rates.y;
            hilstate.yawspeed = (float)earth_rates.z;

            hilstate.vx = (short)(velocity.y * 100); // m/s * 100
            hilstate.vy = (short)(velocity.x * 100); // m/s * 100
            hilstate.vz = 0; // m/s * 100

            hilstate.xacc = (short)(accelerometer.x * 1000); // (mg)
            hilstate.yacc = (short)(accelerometer.y * 1000); // (mg)
            hilstate.zacc = (short)(accelerometer.z * 1000); // (mg)

            MainV2.comPort.sendPacket(hilstate);
        }
    }
}