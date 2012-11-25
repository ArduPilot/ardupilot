using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega.HIL
{
    public class Wind : Utils
    {
        Wind self;
        public float speed;
        public float direction;
        public float turbulance;
        public double cross_section;
            public double turbulance_time_constant;
            public DateTime tlast;
            public double turbulance_mul;

          //'''a wind generation object//'''
    public Wind (string windstring, double cross_section=0.1) {

        self = this;

        string[] a = windstring.Split(',');
        if (Utils.len(a) != 3)
        {
            return;
            //raise RuntimeError("Expected wind in speed,direction,turbulance form, not %s" % windstring);
        }
        self.speed     = float.Parse(a[0]); //# m/s
        self.direction = float.Parse(a[1]); //# direction the wind is going in
        self.turbulance= float.Parse(a[2]); //# turbulance factor (standard deviation)

        //# the cross-section of the aircraft to wind. This is multiplied by the
        //# difference in the wind and the velocity of the aircraft to give the acceleration
        self.cross_section = cross_section;

        //# the time constant for the turbulance - the average period of the
        //# changes over time
        self.turbulance_time_constant = 5.0;

        //# wind time record
        self.tlast = DateTime.Now;

        //# initial turbulance multiplier
        self.turbulance_mul = 1.0;
    }

        public void current(double deltat, out double speed, out double direction) {
        //'''return current wind speed and direction as a tuple
        //speed is in m/s, direction in degrees
        //'''
        if (deltat == 0) {
            DateTime tnow = DateTime.Now;
            deltat = (tnow - self.tlast).TotalSeconds;
            self.tlast = tnow;
        }

        //# update turbulance random walk
        double w_delta = Utils.sqrt(deltat) * (1.0 - new GaussianRandom().NextGaussian(1.0, self.turbulance));
        w_delta -= (self.turbulance_mul-1.0)*(deltat/self.turbulance_time_constant);
        self.turbulance_mul += w_delta;
        speed = self.speed * (float)Utils.fabs(self.turbulance_mul);

        direction = self.direction;
        }

    //# Calculate drag.
        public Vector3 drag(Vector3 velocity, double deltat = 0)//, testing=None) 
    {
        //'''return current wind force in Earth frame.  The velocity parameter is
        //   a Vector3 of the current velocity of the aircraft in earth frame, m/s//'''

        //# (m/s, degrees) : wind vector as a magnitude and angle.
        double speed, direction;
        self.current(deltat,out speed,out direction);
        //# speed = self.speed
        //# direction = self.direction

        //# Get the wind vector.
        Vector3 w = toVec(speed, Utils.radians(direction));

        double obj_speed = velocity.length();

        //# Compute the angle between the object vector and wind vector by taking
        //# the dot product and dividing by the magnitudes.
        double alpha = 0;
        double d = w.length() * obj_speed;
        if (d == 0) {
            alpha = 0;
        }else{
            int checkme;
            alpha = Utils.acos(((w * velocity).length() / d));
        }

        //# Get the relative wind speed and angle from the object.  Note that the
        //# relative wind speed includes the velocity of the object; i.e., there
        //# is a headwind equivalent to the object's speed even if there is no
        //# absolute wind.
        double rel_speed, beta;
        apparent_wind(speed, obj_speed, alpha,out rel_speed,out beta);

        //# Return the vector of the relative wind, relative to the coordinate
        //# system.
        Vector3 relWindVec = toVec(rel_speed, beta + Utils.atan2(velocity.y, velocity.x));

        //# Combine them to get the acceleration vector.
        return new Vector3( acc(relWindVec.x, drag_force(self, relWindVec.x))
                      , acc(relWindVec.y, drag_force(self, relWindVec.y))
                      , 0 );
    }
//# http://en.wikipedia.org/wiki/Apparent_wind
//#
//# Returns apparent wind speed and angle of apparent wind.  Alpha is the angle
//# between the object and the true wind.  alpha of 0 rads is a headwind; pi a
//# tailwind.  Speeds should always be positive.
    public void apparent_wind(double wind_sp, double obj_speed, double alpha, out double rel_speed, out double beta)
    {
        double delta = wind_sp * Utils.cos(alpha);
        double x = Math.Pow(wind_sp, 2) + Math.Pow(obj_speed, 2) + 2 * obj_speed * delta;
        rel_speed = Utils.sqrt(x);
        if (rel_speed == 0)
        {
            beta = Math.PI;
        }
        else
        {
            beta = acos((delta + obj_speed) / rel_speed);
        }
    }

//# See http://en.wikipedia.org/wiki/Drag_equation
//#
//# Drag equation is F(a) = cl * p/2 * v^2 * a, where cl : drag coefficient
//# (let's assume it's low, .e.g., 0.2), p : density of air (assume about 1
//# kg/m^3, the density just over 1500m elevation), v : relative speed of wind
//# (to the body), a : area acted on (this is captured by the cross_section
//# paramter).
//# 
//# So then we have 
//# F(a) = 0.2 * 1/2 * v^2 * cross_section = 0.1 * v^2 * cross_section
public double drag_force(Wind wind, double sp){ 
    return (Math.Pow(sp,2.0)) * 0.1 * wind.cross_section;
}

//# Function to make the force vector.  relWindVec is the direction the apparent
//# wind comes *from*.  We want to compute the accleration vector in the direction
//# the wind blows to.
public double acc(double val,double mag){
    if (val == 0) {
        return mag;
    }else{
        return (val / Utils.fabs(val)) * (0 - mag);
    }
}
//# Converts a magnitude and angle (radians) to a vector in the xy plane.
public Vector3 toVec(double magnitude, double angle)
{
    Vector3 v = new Vector3(magnitude, 0, 0);
    Matrix3 m = new Matrix3();
    m.from_euler(0, 0, angle);
    return m.transposed() * v;
}
    }

    public sealed class GaussianRandom
    {
        private bool _hasDeviate;
        private double _storedDeviate;
        private readonly Random _random;

        public GaussianRandom(Random random = null)
        {
            _random = random ?? new Random();
        }

        /// <summary>
        /// Obtains normally (Gaussian) distributed random numbers, using the Box-Muller
        /// transformation.  This transformation takes two uniformly distributed deviates
        /// within the unit circle, and transforms them into two independently
        /// distributed normal deviates.
        /// </summary>
        /// <param name="mu">The mean of the distribution.  Default is zero.</param>
        /// <param name="sigma">The standard deviation of the distribution.  Default is one.</param>
        /// <returns></returns>
        public double NextGaussian(double mu = 0, double sigma = 1)
        {
            if (sigma <= 0)
                throw new ArgumentOutOfRangeException("sigma", "Must be greater than zero.");

            if (_hasDeviate)
            {
                _hasDeviate = false;
                return _storedDeviate * sigma + mu;
            }

            double v1, v2, rSquared;
            do
            {
                // two random values between -1.0 and 1.0
                v1 = 2 * _random.NextDouble() - 1;
                v2 = 2 * _random.NextDouble() - 1;
                rSquared = v1 * v1 + v2 * v2;
                // ensure within the unit circle
            } while (rSquared >= 1 || rSquared == 0);

            // calculate polar tranformation for each deviate
            var polar = Math.Sqrt(-2 * Math.Log(rSquared) / rSquared);
            // store first deviate
            _storedDeviate = v2 * polar;
            _hasDeviate = true;
            // return second deviate
            return v1 * polar * sigma + mu;
        }
    }
}
