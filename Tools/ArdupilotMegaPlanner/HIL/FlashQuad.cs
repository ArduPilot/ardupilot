using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Sharp3D.Math.Core;

namespace ArdupilotMega.HIL
{
    class FlashQuad
    {
        		public ahrsc ahrs	= new ahrsc()			;//		:AHRS;
		public Vector3D loc	= new Vector3D()	;//				:Location;
		public param g			= new param()	;//		:Parameters;
		//public var apm_rc					:APM_RC;
		//public var motor_filter_0			:AverageFilter;
		//public var motor_filter_1			:AverageFilter;
		//public var motor_filter_2			:AverageFilter;
		//public var motor_filter_3			:AverageFilter;
        public Vector3D drag = new Vector3D();//					:Vector3D;		//
        public Vector3D airspeed = new Vector3D();//					:Vector3D;		//
		//public var thrust					:Vector3D;		//
        public Vector3D position = new Vector3D();//					:Vector3D;		//
        public Vector3D velocity = new Vector3D();//					:Vector3D;
		//public var velocity_old				:Vector3D;
		//public var wind						:Point;			//
        public Vector3D rot_accel = new Vector3D();//				:Vector3D;			//
        public Vector3D angle3D = new Vector3D();//				:Vector3D;			//
		//public var windGenerator			:Wind;			//

		public double gravity					 	= 980.5;
		public double thrust_scale				 	= 0.4;
		public double throttle							= 500;
		public double rotation_bias			 	= 1;

        private double _jump_z					 	= 0;
        private Vector3D v3 = new Vector3D();

        public class ahrsc
        {
            public Matrix3D dcm = new Matrix3D(Matrix3D.Identity);
            public Vector3D gyro = new Vector3D();
            public Vector3D omega = new Vector3D();
            public Vector3D accel = new Vector3D();
            public Vector3D rotation_speed = new Vector3D();

            public double roll_sensor;
            public double pitch_sensor;
            public double yaw_sensor;
        }

        public class param
        {
            // ---------------------------------------------
            // Sim Details controls
            // ---------------------------------------------
            public int sim_iterations = 99999;
            public int sim_speed = 1;

            public double windSpeedMin = 150;
            public double windSpeedMax = 200;
            public double windPeriod = 30000;
            public double windDir = 45;

            public double airDensity = 1.184;
            //public var crossSection				:Number 	= 0.015;
            public double crossSection = 0.012;
            public double dragCE = 0.20;
            public double speed_filter_size = 2;
            public double motor_kv = 1000;
            public double moment = 3;
            public double mass = 500;
            public int esc_delay = 12;

            // -----------------------------------------
            // Inertial control
            // -----------------------------------------
            public double speed_correction_z = 0.0350;
            public double xy_speed_correction = 0.030;
            public double xy_offset_correction = 0.00001;
            public double xy_pos_correction = 0.08;

            public double z_offset_correction = 0.00004;
            public double z_pos_correction = 0.2;

            public double accel_bias_x = .9;
            public double accel_bias_z = .9;
            public double accel_bias_y = .9;
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
            return v * 1000;
        }

        public void update(ref double[] motor_output, double dt)
		{
			var _thrust	= 0.0;
			rot_accel = new Vector3D(0,0,0);
			angle3D.X = angle3D.Y = 0;
			angle3D.Z = 1;

		//	wind = windGenerator.read();

			// ESC's moving average filter
		//	var motor_output:Array = new Array(4);
		//	motor_output[0] = motor_filter_0.apply(apm_rc.get_motor_output(0));
		//	motor_output[1] = motor_filter_1.apply(apm_rc.get_motor_output(1));
		//	motor_output[2] = motor_filter_2.apply(apm_rc.get_motor_output(2));
		//	motor_output[3] = motor_filter_3.apply(apm_rc.get_motor_output(3));


            for (int i = 0; i < motor_output.Length; i++)
            {
                if (motor_output[i] <= 0.0)
                {
                    motor_output[i] = 0;
                }
                else
                {
                    motor_output[i] = scale_rc(i, (float)motor_output[i], 0.0f, 1.0f);
                    //servos[i] = motor_speed[i];
                }
            }

/*
		2

	1		0

		3

*/

			// setup motor rotations
			rot_accel.X 		-= g.motor_kv  * motor_output[0]; // roll
			rot_accel.X  		+= g.motor_kv  * motor_output[1];
			rot_accel.Y  		-= g.motor_kv  * motor_output[3];
			rot_accel.Y 		+= g.motor_kv  * motor_output[2];

			rot_accel.Z  		+= g.motor_kv  * motor_output[0] * .08; // YAW
			rot_accel.Z  		+= g.motor_kv  * motor_output[1] * .08;
			rot_accel.Z  		-= g.motor_kv  * motor_output[2] * .08;
			rot_accel.Z  		-= g.motor_kv  * motor_output[3] * .08;

			rot_accel.X 		/= g.moment;
			rot_accel.Y 		/= g.moment;
			rot_accel.Z 		/= g.moment;

    		//# rotational air resistance

			// Gyro is the rotation speed in deg/s
			// update rotational rates in body frame
			ahrs.gyro.X	+= rot_accel.X * dt;
			ahrs.gyro.Y	+= rot_accel.Y * dt;
			ahrs.gyro.Z	+= rot_accel.Z * dt;

			//ahrs.gyro.z	+= 200;
			ahrs.gyro.Z	*= .995;// some drag

          //  ahrs.dcm = Matrix3D.Identity;

			// move earth frame to body frame
			Vector3D tmp = Matrix3D.Transform(ahrs.dcm,ahrs.gyro) ;//ahrs.dcm.transformVector(ahrs.gyro);

			// update attitude:
            ahrs.dcm += new Matrix3D(new Vector3D((tmp.X/100) * dt,0,0),new Vector3D(0,(tmp.Y/100) * dt,0),new Vector3D(0,0,(tmp.Z/100) * dt));

			//ahrs.dcm.appendRotation((tmp.X/100) * dt, 	Vector3D.X_AXIS);	// ROLL
			//ahrs.dcm.appendRotation((tmp.Y/100) * dt, 	Vector3D.Y_AXIS); 	// PITCH
			//ahrs.dcm.appendRotation((tmp.Z/100) * dt, 	Vector3D.Z_AXIS);	// YAW

			// ------------------------------------
			// calc thrust
			// ------------------------------------

			//get_motor_output returns 0 : 1000
			_thrust += motor_output[0] * thrust_scale;
			_thrust += motor_output[1] * thrust_scale;
			_thrust += motor_output[2] * thrust_scale;
			_thrust += motor_output[3] * thrust_scale;

			Vector3D accel_body 	= new Vector3D(0, 0, (_thrust * -.9) / g.mass);
			//var accel_body:Vector3D 	= new Vector3D(0, 0, 0);
			Vector3D accel_earth	= Matrix3D.Transform(ahrs.dcm,(accel_body));
			angle3D						= Matrix3D.Transform(ahrs.dcm,(angle3D));

			//trace(ahrs.gyro.y, accel_earth.x);

			//trace(ahrs.gyro.x, ahrs.gyro.y, ahrs.gyro.z);

			// ------------------------------------
			// calc copter velocity
			// ------------------------------------
			// calc Drag
			drag.X = .5 * g.airDensity * airspeed.X * airspeed.X * g.dragCE * g.crossSection;
			drag.Y = .5 * g.airDensity * airspeed.Y * airspeed.Y * g.dragCE * g.crossSection;
			drag.Z = .5 * g.airDensity * airspeed.Z * airspeed.Z * g.dragCE * g.crossSection;

			///*
			// this calulation includes wind
			if(airspeed.X >= 0)
				accel_earth.X 	-= drag.X;
			else
				accel_earth.X 	+= drag.X;

			// Add in Drag
			if(airspeed.Y >= 0)
				accel_earth.Y 	-= drag.Y;
			else
				accel_earth.Y 	+= drag.Y;

			if(airspeed.Z <= 0)
				accel_earth.Z 	-= drag.Z;
			else
				accel_earth.Z 	+= drag.Z;
			//*/

			// hacked vert disturbance
			accel_earth.Z	+= _jump_z * dt;
			_jump_z 		*= .999;


			// Add in Gravity
			accel_earth.Z += gravity;

			if(accel_earth.Z < 0)
				accel_earth.Z *=.9;


			if(position.Z <=.11 && accel_earth.Z > 0){
				accel_earth.Z = 0;
			}

			velocity.X 		+= (accel_earth.X * dt); // + : Forward (North)
			velocity.Y  	+= (accel_earth.Y * dt); // + : Right (East)
			velocity.Z  	-= (accel_earth.Z * dt); // + : Up


			//trace(Math.floor(velocity.x),Math.floor(velocity.y),Math.floor(velocity.z));

			// ------------------------------------
			// calc inertia
			// ------------------------------------

			// work out acceleration as seen by the accelerometers. It sees the kinematic
			// acceleration (ie. real movement), plus gravity
			Matrix3D dcm_t			= ahrs.dcm.Clone();
			dcm_t.Transpose();
			double t = accel_earth.Z;
			accel_earth.Z -= gravity;
			ahrs.accel = Matrix3D.Transform(dcm_t,(accel_earth));

			ahrs.accel *= 0.01;

			//ahrs.accel	= accel_earth.clone();
			ahrs.accel.X	*= g.accel_bias_x;
			ahrs.accel.Y	*= g.accel_bias_y;
			ahrs.accel.Z	*= g.accel_bias_z;



			// ------------------------------------
			// calc Position
			// ------------------------------------
			position.Y 		+= velocity.X * dt;
			position.X 		+= velocity.Y * dt;
			position.Z  	+= velocity.Z * dt;
			position.Z 		= Math.Min(position.Z, 4000);

			// XXX Force us to 3m above ground
			//position.z = 300;

			airspeed.X  	= (velocity.X);// - wind.x);
			airspeed.Y  	= (velocity.Y);// - wind.y);
			airspeed.Z  	= velocity.Z;

			// Altitude
			// --------
			if(position.Z <=.1){
				position.Z 	= .1;
				velocity.X 	= 0;
				velocity.Y 	= 0;
				velocity.Z 	= 0;
                airspeed.X = 0;
                airspeed.Y = 0;
                airspeed.Z = 0;
				//ahrs.init();
			}


			// get omega - the simulated Gyro output
			ahrs.omega.X 		= radiansx100(ahrs.gyro.X);
			ahrs.omega.Y 		= radiansx100(ahrs.gyro.Y);
			ahrs.omega.Z 		= radiansx100(ahrs.gyro.Z);

			// get the Eulers output
            v3 = new Vector3D(Math.Atan2(-ahrs.dcm.M23, ahrs.dcm.M22), Math.Asin(ahrs.dcm.M21), Math.Atan2(-ahrs.dcm.M31, ahrs.dcm.M11));// ahrs.dcm.decompose();
			ahrs.roll_sensor 	=  Math.Floor(degrees(v3.X) * 100);
			ahrs.pitch_sensor 	=  Math.Floor(degrees(v3.Y) * 100);
			ahrs.yaw_sensor 	=  Math.Floor(degrees(v3.Z) * 100);

			// store the position for the GPS object
			loc.X = position.X;
			loc.Y = position.Y;
			loc.Z = position.Z;
		}

        public double constrain(double val, double min, double max)
        {
            val = Math.Max(val, min);
            val = Math.Min(val, max);
            return val;
        }

		public double wrap_180(int error)
		{
			if (error > 18000)	error -= 36000;
			if (error < -18000)	error += 36000;
			return error;
		}

		public int wrap_360(int error)
		{
			if (error > 36000)	error -= 36000;
			if (error < 0)		error += 36000;
			return error;
		}

		public double radiansx100(double n)
		{
			return 0.000174532925 * n;
		}

		public double degreesx100(double r)
		{
			return r * 5729.57795;
		}
		public double degrees(double r)
		{
			return r * 57.2957795;
		}
		public double radians(double n)
		{
			return 0.0174532925 * n;
		}
    }
}
