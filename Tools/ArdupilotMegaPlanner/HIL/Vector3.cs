using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega.HIL
{
    public class Vector3
    {
        Vector3 self;
        public double x;
        public double y;
        public double z;

        public Vector3(double x = 0, double y = 0, double z = 0) 
        {
            self = this;
            this.x = x;
            this.y = y;
            this.z = z;
        }

    public new string ToString() {
        return String.Format("Vector3({0}, {1}, {2})",self.x,
                                              self.y,
                                              self.z);
    }

    public static Vector3 operator +(Vector3 self, Vector3 v) {

        return new Vector3(self.x + v.x,
                       self.y + v.y,
                       self.z + v.z);
}


    public static Vector3 operator -(Vector3 self, Vector3 v) {
        return new Vector3(self.x - v.x,
                       self.y - v.y,
                       self.z - v.z);
    }

    public static Vector3 operator -(Vector3 self) {
        return new Vector3(-self.x, -self.y, -self.z);
    }
        

    public static Vector3 operator *(Vector3 self, Vector3 v) {
          //  '''dot product'''
            return new Vector3(self.x*v.x + self.y*v.y + self.z*v.z);

    }

        public static Vector3 operator *(Vector3 self, double v) {
                 return new Vector3(self.x * v,
                       self.y * v,
                       self.z * v);
        }

        public static Vector3 operator *(double v, Vector3 self)
        {
            return (self * v);
        }

    public static Vector3 operator /(Vector3 self, double v) {
        return new Vector3(self.x / v,
                       self.y / v,
                       self.z / v);
    }

    public static Vector3 operator %(Vector3 self, Vector3 v) {
      //  '''cross product'''
        return new Vector3(self.y*v.z - self.z*v.y,
                       self.z*v.x - self.x*v.z,
                       self.x*v.y - self.y*v.x);
    }

    public Vector3 copy() {
        return new Vector3(self.x, self.y, self.z);
    }


    public double length() {
        return Math.Sqrt(self.x*self.x + self.y*self.y + self.z*self.z);
    }

    public void zero() {
        self.x = self.y = self.z = 0;
    }

   //public double angle (Vector3 self, Vector3 v) {
     //   '''return the angle between this vector and another vector'''
      //  return Math.Acos(self * v) / (self.length() * v.length());
    //}

    public Vector3 normalized(){
        return self / self.length();
    }

    public void normalize() {
        Vector3 v = self.normalized();
        self.x = v.x;
        self.y = v.y;
        self.z = v.z;
    }
    }
}
