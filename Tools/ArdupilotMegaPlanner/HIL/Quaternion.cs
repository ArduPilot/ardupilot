using System;
using System.Collections.Generic;
using System.Text;

namespace YLScsDrawing.Drawing3d
{
    public struct Quaternion
    {
        public double X, Y, Z, W;

        public Quaternion(double w, double x, double y, double z)
        {
            W = w; X = x; Y = y; Z = z;
        }

        public Quaternion(double w, Vector3d v)
        {
            W = w; X = v.X; Y = v.Y; Z = v.Z;
        }

        public Vector3d V
        {
            set { X = value.X; Y = value.Y; Z = value.Z; }
            get { return new Vector3d(X, Y, Z); }
        }

        public void Normalise()
        {
            double m = W * W + X * X + Y * Y + Z * Z;
            if (m > 0.001)
            {
                m = Math.Sqrt(m);
                W /= m;
                X /= m;
                Y /= m;
                Z /= m;
            }
            else
            {
                W = 1; X = 0; Y = 0; Z = 0;
            }
        }

        public void Conjugate()
        {
            X = -X; Y = -Y; Z = -Z;
        }

        public void FromAxisAngle(Vector3d axis, double angleRadian)
        {
            double m = axis.Magnitude;
            if (m > 0.0001)
            {
                double ca = Math.Cos(angleRadian / 2);
                double sa = Math.Sin(angleRadian / 2);
                X = axis.X / m * sa;
                Y = axis.Y / m * sa;
                Z = axis.Z / m * sa;
                W = ca;
            }
            else
            {
                W = 1; X = 0; Y = 0; Z = 0;
            }
        }

        public Quaternion Copy()
        {
            return new Quaternion(W, X, Y, Z);
        }

        public void Multiply(Quaternion q)
        {
            this *= q;
        }

        //                  -1
        // V'=q*V*q     ,
        public void Rotate(Point3d pt)
        {
            this.Normalise();
            Quaternion q1 = this.Copy();
            q1.Conjugate();

            Quaternion qNode = new Quaternion(0, pt.X, pt.Y, pt.Z);
            qNode = this * qNode * q1;
            pt.X = qNode.X;
            pt.Y = qNode.Y;
            pt.Z = qNode.Z;
        }

        public void Rotate(Point3d[] nodes)
        {
            this.Normalise();
            Quaternion q1 = this.Copy();
            q1.Conjugate();
            for (int i = 0; i < nodes.Length; i++)
            {
                Quaternion qNode = new Quaternion(0, nodes[i].X, nodes[i].Y, nodes[i].Z);
                qNode = this * qNode * q1;
                nodes[i].X = qNode.X;
                nodes[i].Y = qNode.Y;
                nodes[i].Z = qNode.Z;
            }
        }

        // Multiplying q1 with q2 is meaning of doing q2 firstly then q1
        public static Quaternion operator *(Quaternion q1, Quaternion q2)
        {
            double nw = q1.W * q2.W - q1.X * q2.X - q1.Y * q2.Y - q1.Z * q2.Z;
            double nx = q1.W * q2.X + q1.X * q2.W + q1.Y * q2.Z - q1.Z * q2.Y;
            double ny = q1.W * q2.Y + q1.Y * q2.W + q1.Z * q2.X - q1.X * q2.Z;
            double nz = q1.W * q2.Z + q1.Z * q2.W + q1.X * q2.Y - q1.Y * q2.X;
            return new Quaternion(nw, nx, ny, nz);
        }

        public static Vector3d operator *(Quaternion self, Vector3d other)
        {
            double w = self.W;
            double x = self.X;
            double y = self.Y;
            double z = self.Z;
            double Vx = other.X;
            double Vy = other.Y;
            double Vz = other.Z;
            double ww = w * w;
            double w2 = w * 2;
            double wx2 = w2 * x;
            double wy2 = w2 * y;
            double wz2 = w2 * z;
            double xx = x * x;
            double x2 = x * 2;
            double xy2 = x2 * y;
            double xz2 = x2 * z;
            double yy = y * y;
            double yz2 = 2 * y * z;
            double zz = z * z;
            return new Vector3d(
               ww * Vx + wy2 * Vz - wz2 * Vy + 
               xx * Vx + xy2 * Vy + xz2 * Vz - 
               zz * Vx - yy * Vx,
               xy2 * Vx + yy * Vy + yz2 * Vz + 
               wz2 * Vx - zz * Vy + ww * Vy - 
               wx2 * Vz - xx * Vy,
               xz2 * Vx + yz2 * Vy + 
               zz * Vz - wy2 * Vx - yy * Vz + 
               wx2 * Vy - xx * Vz + ww * Vz);
                }
    }
}
