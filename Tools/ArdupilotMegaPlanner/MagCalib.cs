using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;

namespace ArdupilotMega
{
    public class MagCalib
    {

        //alglib.lsfit.

        public static void doWork()
        {
            /*
            double[,] x = new double[,] { { -1 }, { -0.8 }, { -0.6 }, { -0.4 }, { -0.2 }, { 0 }, { 0.2 }, { 0.4 }, { 0.6 }, { 0.8 }, { 1.0 } };
            double[] y = new double[] { 0.223130, 0.382893, 0.582748, 0.786628, 0.941765, 1.000000, 0.941765, 0.786628, 0.582748, 0.382893, 0.223130 };
            double[] c = new double[] { 0.3 };
            double epsf = 0;
            double epsx = 0.000001;
            int maxits = 0;
            int info;
            alglib.lsfitstate state;
            alglib.lsfitreport rep;
            double diffstep = 0.0001;

            //
            // Fitting without weights
            //
            alglib.lsfitcreatef(x, y, c, diffstep, out state);
            alglib.lsfitsetcond(state, epsf, epsx, maxits);
            alglib.lsfitfit(state, function_cx_1_func, null, null);
            alglib.lsfitresults(state, out info, out c, out rep);
            System.Console.WriteLine("{0}", info); // EXPECTED: 2
            System.Console.WriteLine("{0}", alglib.ap.format(c, 1)); // EXPECTED: [1.5]
            */

            // based of tridge's work

            Tuple<float, float, float> offset = new Tuple<float, float, float>(0, 0, 0);
            List<Tuple<float, float, float>> data = new List<Tuple<float, float, float>>();


            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.Filter = "*.tlog|*.tlog";
            openFileDialog1.FilterIndex = 2;
            openFileDialog1.RestoreDirectory = true;
            openFileDialog1.Multiselect = true;
            try
            {
                openFileDialog1.InitialDirectory = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs" + Path.DirectorySeparatorChar;
            }
            catch { } // incase dir doesnt exist

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                foreach (string logfile in openFileDialog1.FileNames)
                {

                    MAVLink mine = new MAVLink();
                    mine.logplaybackfile = new BinaryReader(File.Open(logfile, FileMode.Open, FileAccess.Read, FileShare.Read));
                    mine.logreadmode = true;

                    mine.packets.Initialize(); // clear

                    // gather data
                    while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length)
                    {
                        // bar moves to 100 % in this step
                        //progressBar1.Value = (int)((float)mine.logplaybackfile.BaseStream.Position / (float)mine.logplaybackfile.BaseStream.Length * 100.0f / 1.0f);

                        //progressBar1.Refresh();
                        //Application.DoEvents();

                        byte[] packet = mine.readPacket();

                        var pack = mine.DebugPacket(packet);

                        if (pack.GetType() == typeof(MAVLink.__mavlink_sensor_offsets_t))
                        {
                            offset = new Tuple<float,float,float>(
                                ((MAVLink.__mavlink_sensor_offsets_t)pack).mag_ofs_x,
                                ((MAVLink.__mavlink_sensor_offsets_t)pack).mag_ofs_y,
                                ((MAVLink.__mavlink_sensor_offsets_t)pack).mag_ofs_z);
                        }
                        else if (pack.GetType() == typeof(MAVLink.__mavlink_raw_imu_t))
                        {
                            data.Add(new Tuple<float, float, float>(
                                ((MAVLink.__mavlink_raw_imu_t)pack).xmag - offset.Item1,
                                ((MAVLink.__mavlink_raw_imu_t)pack).ymag - offset.Item2,
                                ((MAVLink.__mavlink_raw_imu_t)pack).zmag - offset.Item3));
                        }

                    }


    



                    //progressBar1.Value = 100;

                    mine.logreadmode = false;
                    mine.logplaybackfile.Close();
                    mine.logplaybackfile = null;

                }
            }
        }

        public static List<double> sphere_error(double[,] p, double[] data)
        {
            double xofs = p[0, 0];
            double yofs = p[0, 1];
            double zofs = p[0, 2];
            double r = p[0, 3];
            List<double> ret = new List<double>();
            foreach (var d in data)
            {
                //double x, y, z = d;
                //double err = r - Math.Sqrt(Math.Pow((x + xofs), 2) + Math.Pow((y + yofs), 2) + Math.Pow((z + zofs), 2));
                //ret.Add(err);
            }
            return ret;
        }

        public static void function_cx_1_func(double[] c, double[] x, ref double func, object obj)
        {
            // this callback calculates f(c,x)=exp(-c0*sqr(x0))
            // where x is a position on X-axis and c is adjustable parameter
            func = System.Math.Exp(-c[0] * x[0] * x[0]);
        }


    }
}