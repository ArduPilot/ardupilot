using System;
using System.Collections.Generic;
using System.Drawing;

namespace YLScsDrawing.Drawing3d
{
    public struct Point3d
    {
        public double X, Y, Z; // coordinate system follows right-hand rule

        public Point3d(double x, double y, double z)
        {
            X = x; Y = y; Z = z;
        }

        public Point3d(Vector3d v)
        {
            X = v.X; Y = v.Y; Z = v.Z;
        }

        public Point3d Copy()
        {
            return new Point3d(this.X, this.Y, this.Z);
        }

        public Vector3d ToVector3d()
        {
            return new Vector3d(X, Y, Z);
        }

        public void Offset(double x, double y, double z)
        {
            this.X += x;
            this.Y += y;
            this.Z += z;
        }

        public static Point3d[] Copy(Point3d[] pts)
        {
            Point3d[] copy = new Point3d[pts.Length];
            for (int i = 0; i < pts.Length; i++)
            {
                copy[i] = pts[i].Copy();
            }
            return copy;
        }

        public static void Offset(Point3d[] pts, double offsetX, double offsetY, double offsetZ)
        {
            for (int i = 0; i < pts.Length; i++)
            {
                pts[i].Offset(offsetX, offsetY, offsetZ);
            }
        }

        public PointF GetProjectedPoint(double d /* project distance: from eye to screen*/)
        {
            return new PointF((float)(this.X * d / (d + this.Z)), (float)(this.Y * d / (d + this.Z)));
        }

        public static PointF[] Project(Point3d[] pts, double d /* project distance: from eye to screen*/)
        {
            PointF[] pt2ds = new PointF[pts.Length];
            for (int i = 0; i < pts.Length; i++)
            {
                pt2ds[i] = pts[i].GetProjectedPoint(d);
            }
            return pt2ds;
        }
    }
}

