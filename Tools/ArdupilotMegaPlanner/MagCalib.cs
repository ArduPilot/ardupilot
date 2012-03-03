using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.Collections;

namespace ArdupilotMega
{
    public class MagCalib
    {

        //alglib.lsfit.

        public static void doWork()
        {
            // based of tridge's work

            Tuple<float, float, float> offset = new Tuple<float, float, float>(0, 0, 0);
            List<Tuple<float, float, float>> data = new List<Tuple<float, float, float>>();

            Hashtable filter = new Hashtable();

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

            openFileDialog1.FileName = @"C:\Users\hog\Downloads\2012-02-05.log";

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

                        byte[] packetraw = mine.readPacket();

                        var packet = mine.DebugPacket(packetraw);

                        if (packet == null)
                            continue;

                        if (packet.GetType() == typeof(MAVLink.__mavlink_sensor_offsets_t))
                        {
                            offset = new Tuple<float,float,float>(
                                ((MAVLink.__mavlink_sensor_offsets_t)packet).mag_ofs_x,
                                ((MAVLink.__mavlink_sensor_offsets_t)packet).mag_ofs_y,
                                ((MAVLink.__mavlink_sensor_offsets_t)packet).mag_ofs_z);
                        }
                        else if (packet.GetType() == typeof(MAVLink.__mavlink_raw_imu_t))
                        {
                            int div = 20;

                            string item = (int)(((MAVLink.__mavlink_raw_imu_t)packet).xmag / div) + "," +
                                (int)(((MAVLink.__mavlink_raw_imu_t)packet).ymag / div) + "," +
                                (int)(((MAVLink.__mavlink_raw_imu_t)packet).zmag / div);

                            if (filter.ContainsKey(item))
                            {
                                filter[item] = (int)filter[item] + 1;

                                if ((int)filter[item] > 3)
                                    continue;
                            }
                            else
                            {
                                filter[item] = 1;
                            }
                            

                            data.Add(new Tuple<float, float, float>(
                                ((MAVLink.__mavlink_raw_imu_t)packet).xmag - offset.Item1,
                                ((MAVLink.__mavlink_raw_imu_t)packet).ymag - offset.Item2,
                                ((MAVLink.__mavlink_raw_imu_t)packet).zmag - offset.Item3));
                        }

                    }

                    Console.WriteLine("Extracted " + data.Count + " data points");
                    Console.WriteLine("Current offset: " + offset);

                    mine.logreadmode = false;
                    mine.logplaybackfile.Close();
                    mine.logplaybackfile = null;

                    double[] x = new double[] { 0, 0, 0, 0 };
                    double epsg = 0.0000000001;
                    double epsf = 0;
                    double epsx = 0;
                    int maxits = 0;
                    alglib.minlmstate state;
                    alglib.minlmreport rep;

                    alglib.minlmcreatev(data.Count, x, 100, out state);
                    alglib.minlmsetcond(state, epsg, epsf, epsx, maxits);
                    alglib.minlmoptimize(state, sphere_error, null, data);
                    alglib.minlmresults(state, out x, out rep);

                    System.Console.WriteLine("{0}", rep.terminationtype); // EXPECTED: 4
                    System.Console.WriteLine("{0}", alglib.ap.format(x, 2)); // EXPECTED: [-3,+3]
                    //System.Console.ReadLine();


                  //  return;


                }
            }
        }

        public static void sphere_error(double[] xi, double[] fi, object obj)
        {
            double xofs = xi[0];
            double yofs = xi[1];
            double zofs = xi[2];
            double r = xi[3];
            int a = 0;
            foreach (var d in (List<Tuple<float, float, float>>)obj)
            {
                double x = d.Item1;
                double y = d.Item2;
                double z = d.Item3;
                double err = r - Math.Sqrt(Math.Pow((x + xofs), 2) + Math.Pow((y + yofs), 2) + Math.Pow((z + zofs), 2));
                fi[a] = err;
                a++;
            }
        }
    }
}