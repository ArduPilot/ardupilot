using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega.HIL
{
    public class Utils
    {
        public const float rad2deg = (float)(180 / System.Math.PI);
        public const float deg2rad = (float)(1.0 / rad2deg);
        public const float ft2m = (float)(1.0 / 3.2808399);
        public const float kts2fps = (float)1.68780986;

        public static double sin(double val)
        {
            return System.Math.Sin(val);
        }
        public static double cos(double val)
        {
            return System.Math.Cos(val);
        }
        public static double radians(double val)
        {
            return val * deg2rad;
        }
                public static double degrees(double val)
        {
            return val * rad2deg;
        }
        public static double sqrt(double val)
        {
            return System.Math.Sqrt(val);
        }

        public static int[] range(int no)
        {
            int[] range = new int[no];
            for (int a = 0; a < no; a++)
            {
                range[a] = a;
            }
            return range;
        }

        public static double fabs(double val)
        {
            return System.Math.Abs(val);
        }

        public static double tan(double val)
        {
            return System.Math.Tan(val);
        }

        
public static Tuple<double,double,double> EarthRatesToBodyRates(double roll,double pitch,double yaw,
                         double rollRate,double pitchRate,double yawRate) {
    //convert the angular velocities from earth frame to
    //body frame. Thanks to James Goppert for the formula

    //all inputs and outputs are in degrees

    //returns a tuple, (p,q,r)
   

    var phi      = radians(roll);
    var theta    = radians(pitch);
    var phiDot   = radians(rollRate);
    var thetaDot = radians(pitchRate);
    var psiDot   = radians(yawRate);

    var p = phiDot - psiDot*sin(theta);
    var q = cos(phi)*thetaDot + sin(phi)*psiDot*cos(theta);
    var r = cos(phi)*psiDot*cos(theta) - sin(phi)*thetaDot;

    return new Tuple<double,double,double> (degrees(p), degrees(q), degrees(r));
}

public static Tuple<double,double,double> BodyRatesToEarthRates(double roll, double pitch, double yaw,double  pDeg, double qDeg,double  rDeg){
    //convert the angular velocities from body frame to
    //earth frame.

    //all inputs and outputs are in degrees

    //returns a tuple, (rollRate,pitchRate,yawRate)

    var p      = radians(pDeg);
    var q      = radians(qDeg);
    var r      = radians(rDeg);

    var phi      = radians(roll);
    var theta    = radians(pitch);

    var phiDot   = p + tan(theta)*(q*sin(phi) + r*cos(phi));
    var thetaDot = q*cos(phi) - r*sin(phi);
    if (fabs(cos(theta)) < 1.0e-20)
        theta += 1.0e-10;
    var psiDot   = (q*sin(phi) + r*cos(phi))/cos(theta);

    return new Tuple<double,double,double> (degrees(phiDot), degrees(thetaDot), degrees(psiDot));
    }
    }
}
