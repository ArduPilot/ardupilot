using System;
using System.Collections.Generic;
using System.Text;

namespace YLScsDrawing.Drawing3d
{
    public struct Vector3d
    {
        public double X, Y, Z;

        public Vector3d(double x, double y, double z)
        {
            X = x; Y = y; Z = z;
        }

        public Vector3d(Vector3d pt)
        {
            X = pt.X; Y = pt.Y; Z = pt.Z;
        }

        public Vector3d(Point3d pt)
        {
            X = pt.X; Y = pt.Y; Z = pt.Z;
        }

        public Vector3d(Point3d startPoint, Point3d endPoint)
        {
            X = endPoint.X - startPoint.X;
            Y = endPoint.Y - startPoint.Y;
            Z = endPoint.Z - startPoint.Z;
        }

        public double Magnitude
        {
            get { return Math.Sqrt(X * X + Y * Y + Z * Z); }
        }

        public void Normalise()
        {
            double m = Math.Sqrt(X * X + Y * Y + Z * Z);
            if (m > 0.001)
            {
                X /= m; Y /= m; Z /= m;
            }
        }

        public static Vector3d operator +(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
        }

        public static Vector3d operator -(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
        }

        public static Vector3d operator -(Vector3d v)
        {
            return new Vector3d(-v.X, -v.Y, -v.Z);
        }

        public static Vector3d operator *(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.X * v2.X , v1.Y * v2.Y, v1.Z * v2.Z);
        }

        public static Vector3d operator *(Vector3d v1, double s2)
        {
            return new Vector3d(v1.X * s2, v1.Y * s2, v1.Z * s2);
        }

        // A x B = |A|*|B|*sin(angle), direction follow right-hand rule
        public static Vector3d CrossProduct(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.Y * v2.Z - v1.Z * v2.Y, v1.Z * v2.X - v1.X * v2.Z, v1.X * v2.Y - v1.Y * v2.X);
        }

        public static double DotProduct(Vector3d v1, Vector3d v2) // A . B = |A|*|B|*cos(angle)
        {
            return (v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z);
        }

        public Vector3d CrossProduct(Vector3d v)
        {
            return CrossProduct(this, v);
        }

        public double DotProduct(Vector3d v)
        {
            return DotProduct(this, v);
        }

        public static bool isForeFace(Point3d pt1, Point3d pt2, Point3d pt3) // pts on a plane
        {
            Vector3d v1 = new Vector3d(pt2, pt1);
            Vector3d v2 = new Vector3d(pt2, pt3);
            Vector3d v = v1.CrossProduct(v2);
            return v.DotProduct(new Vector3d(0, 0, 1)) < 0;
        }

        public static bool isBackFace(Point3d pt1, Point3d pt2, Point3d pt3)
        {
            Vector3d v1 = new Vector3d(pt2, pt1);
            Vector3d v2 = new Vector3d(pt2, pt3);
            Vector3d v = v1.CrossProduct(v2);
            return v.DotProduct(new Vector3d(0, 0, 1)) > 0;
        }
    }
}