using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using YLScsDrawing.Drawing3d;

namespace ArdupilotMega.HIL
{
    public class Matrix3
    {
        //   '''a 3x3 matrix, intended as a rotation matrix'''

        Matrix3 self;
        Vector3 a;
        Vector3 b;
        Vector3 c;

        public Matrix3()
        {
            self = this;
            self.identity();
        }

        public Matrix3(Vector3 a, Vector3 b, Vector3 c)
        {
            self = this;

            this.a = a;
            this.b = b;
            this.c = c;
        }

        public new string ToString()
        {
            return String.Format("Matrix3(({0}, {1}, {2}), ({3}, {4}, {5}), ({6}, {7}, {8}))",
                self.a.x, self.a.y, self.a.z,
                self.b.x, self.b.y, self.b.z,
                self.c.x, self.c.y, self.c.z);
        }

        public void identity()
        {
            self.a = new Vector3(1, 0, 0);
            self.b = new Vector3(0, 1, 0);
            self.c = new Vector3(0, 0, 1);
        }

        public Matrix3 transposed()
        {
            return new Matrix3(new Vector3(self.a.x, self.b.x, self.c.x),
                           new Vector3(self.a.y, self.b.y, self.c.y),
                           new Vector3(self.a.z, self.b.z, self.c.z));
        }


        public void from_euler(double roll, double pitch, double yaw)
        {
            // '''fill the matrix from Euler angles in radians'''
            double cp = Utils.cos(pitch);
            double sp = Utils.sin(pitch);
            double sr = Utils.sin(roll);
            double cr = Utils.cos(roll);
            double sy = Utils.sin(yaw);
            double cy = Utils.cos(yaw);

            self.a.x = cp * cy;
            self.a.y = (sr * sp * cy) - (cr * sy);
            self.a.z = (cr * sp * cy) + (sr * sy);
            self.b.x = cp * sy;
            self.b.y = (sr * sp * sy) + (cr * cy);
            self.b.z = (cr * sp * sy) - (sr * cy);
            self.c.x = -sp;
            self.c.y = sr * cp;
            self.c.z = cr * cp;
        }

        public void to_euler(ref double roll, ref double pitch, ref double yaw)
        {
            // '''find Euler angles for the matrix'''
            if (self.c.x >= 1.0)
                pitch = Math.PI;
            else if (self.c.x <= -1.0)
                pitch = -Math.PI;
            else
                pitch = -Utils.asin(self.c.x);
            roll = Utils.atan2(self.c.y, self.c.z);
            yaw = Utils.atan2(self.b.x, self.a.x);
            //return (roll, pitch, yaw)
        }

        public static Matrix3 operator +(Matrix3 self, Matrix3 m)
        {
            return new Matrix3(self.a + m.a, self.b + m.b, self.c + m.c);
        }

        public static Matrix3 operator -(Matrix3 self, Matrix3 m)
        {
            return new Matrix3(self.a - m.a, self.b - m.b, self.c - m.c);
        }

        public static Vector3 operator *(Matrix3 self, Vector3 v)
        {
            return new Vector3(self.a.x * v.x + self.a.y * v.y + self.a.z * v.z,
                           self.b.x * v.x + self.b.y * v.y + self.b.z * v.z,
                           self.c.x * v.x + self.c.y * v.y + self.c.z * v.z);
        }

        public static Matrix3 operator *(Matrix3 self, Matrix3 m)
        {
            return new Matrix3(new Vector3(self.a.x * m.a.x + self.a.y * m.b.x + self.a.z * m.c.x,
                                   self.a.x * m.a.y + self.a.y * m.b.y + self.a.z * m.c.y,
                                   self.a.x * m.a.z + self.a.y * m.b.z + self.a.z * m.c.z),
                           new Vector3(self.b.x * m.a.x + self.b.y * m.b.x + self.b.z * m.c.x,
                                   self.b.x * m.a.y + self.b.y * m.b.y + self.b.z * m.c.y,
                                   self.b.x * m.a.z + self.b.y * m.b.z + self.b.z * m.c.z),
                           new Vector3(self.c.x * m.a.x + self.c.y * m.b.x + self.c.z * m.c.x,
                                   self.c.x * m.a.y + self.c.y * m.b.y + self.c.z * m.c.y,
                                   self.c.x * m.a.z + self.c.y * m.b.z + self.c.z * m.c.z));
        }

        public static Matrix3 operator *(Matrix3 self, double v)
        {

            return new Matrix3(self.a * v, self.b * v, self.c * v);
        }

        public static Matrix3 operator /(Matrix3 self, double v)
        {
            return new Matrix3(self.a / v, self.b / v, self.c / v);
        }

        public static Matrix3 operator -(Matrix3 self)
        {
            return new Matrix3(-self.a, -self.b, -self.c);
        }

        public Matrix3 copy()
        {
            return new Matrix3(self.a.copy(), self.b.copy(), self.c.copy());
        }

        public void rotate(Vector3 g)
        {
            //   '''rotate the matrix by a given amount on 3 axes'''
            Matrix3 temp_matrix = new Matrix3(self.a.copy(), self.b.copy(), self.c.copy());

            temp_matrix.a.x = a.y * g.z - a.z * g.y;
            temp_matrix.a.y = a.z * g.x - a.x * g.z;
            temp_matrix.a.z = a.x * g.y - a.y * g.x;
            temp_matrix.b.x = b.y * g.z - b.z * g.y;
            temp_matrix.b.y = b.z * g.x - b.x * g.z;
            temp_matrix.b.z = b.x * g.y - b.y * g.x;
            temp_matrix.c.x = c.y * g.z - c.z * g.y;
            temp_matrix.c.y = c.z * g.x - c.x * g.z;
            temp_matrix.c.z = c.x * g.y - c.y * g.x;
            self.a += temp_matrix.a;
            self.b += temp_matrix.b;
            self.c += temp_matrix.c;
        }

        public void normalize()
        {
            //  '''re-normalise a rotation matrix'''
            Vector3 error = self.a * self.b;
            Vector3 t0 = self.a - (self.b * (0.5 * error));
            Vector3 t1 = self.b - (self.a * (0.5 * error));
            Vector3 t2 = t0 % t1;
            self.a = t0 * (1.0 / t0.length());
            self.b = t1 * (1.0 / t1.length());
            self.c = t2 * (1.0 / t2.length());
        }

        public double trace()
        {
            //  '''the trace of the matrix'''
            return self.a.x + self.b.y + self.c.z;
        }
    }
}
