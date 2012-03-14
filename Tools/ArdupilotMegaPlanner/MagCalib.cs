using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.Collections;
using netDxf;
using netDxf.Entities;
using netDxf.Tables;
using netDxf.Header;
using System.Reflection;
using log4net;

namespace ArdupilotMega
{
    public class MagCalib
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// Self contained process tlog and save/display offsets
        /// </summary>
        public static void ProcessLog()
        {
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
                try
                {
                    double[] ans = getOffsets(openFileDialog1.FileName);

                    SaveOffsets(ans);
                }
                catch (Exception ex) { log.Debug(ex.ToString()); }
            }
        }

        /// <summary>
        /// Processes a tlog to get the offsets - creates dxf of data
        /// </summary>
        /// <param name="fn">Filename</param>
        /// <returns>Offsets</returns>
        public static double[] getOffsets(string fn)
        {
            // based of tridge's work
            string logfile = fn;

            // old method
            float minx = 0;
            float maxx = 0;
            float miny = 0;
            float maxy = 0;
            float minz = 0;
            float maxz = 0;

            // this is for a dxf
            Polyline3dVertex vertex;
            List<Polyline3dVertex> vertexes = new List<Polyline3dVertex>();

            // data storage
            Tuple<float, float, float> offset = new Tuple<float, float, float>(0, 0, 0);
            List<Tuple<float, float, float>> data = new List<Tuple<float, float, float>>();

            Hashtable filter = new Hashtable();

            log.Info("Start log: " + DateTime.Now);

                MAVLink mine = new MAVLink();
                mine.logplaybackfile = new BinaryReader(File.Open(logfile, FileMode.Open, FileAccess.Read, FileShare.Read));
                mine.logreadmode = true;

                mine.packets.Initialize(); // clear

                // gather data
                while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length)
                {
                    byte[] packetraw = mine.readPacket();

                    var packet = mine.DebugPacket(packetraw, false);

                    // this is for packets we dont know about
                    if (packet == null)
                        continue;

                    if (packet.GetType() == typeof(MAVLink.__mavlink_sensor_offsets_t))
                    {
                        offset = new Tuple<float, float, float>(
                            ((MAVLink.__mavlink_sensor_offsets_t)packet).mag_ofs_x,
                            ((MAVLink.__mavlink_sensor_offsets_t)packet).mag_ofs_y,
                            ((MAVLink.__mavlink_sensor_offsets_t)packet).mag_ofs_z);
                    }
                    else if (packet.GetType() == typeof(MAVLink.__mavlink_raw_imu_t))
                    {
                        int div = 20;

                        // fox dxf
                        vertex = new Polyline3dVertex(new Vector3f(
                            ((MAVLink.__mavlink_raw_imu_t)packet).xmag - offset.Item1,
                            ((MAVLink.__mavlink_raw_imu_t)packet).ymag - offset.Item2,
                            ((MAVLink.__mavlink_raw_imu_t)packet).zmag - offset.Item3)
                            );
                        vertexes.Add(vertex);


                        // for old method
                        setMinorMax(((MAVLink.__mavlink_raw_imu_t)packet).xmag - offset.Item1, ref minx, ref maxx);
                        setMinorMax(((MAVLink.__mavlink_raw_imu_t)packet).ymag - offset.Item2, ref miny, ref maxy);
                        setMinorMax(((MAVLink.__mavlink_raw_imu_t)packet).zmag - offset.Item3, ref minz, ref maxz);

                        // for new lease sq
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

                log.Info("Log Processed " + DateTime.Now);

                Console.WriteLine("Extracted " + data.Count + " data points");
                Console.WriteLine("Current offset: " + offset);

                mine.logreadmode = false;
                mine.logplaybackfile.Close();
                mine.logplaybackfile = null;

                if (data.Count < 10)
                {
                    CustomMessageBox.Show("Log does not contain enough data");
                    throw new Exception("Not Enough Data");
                }

                double[] x = LeastSq(data);

                System.Console.WriteLine("Old Method {0} {1} {2}", -(maxx + minx) / 2, -(maxy + miny) / 2, -(maxz + minz) / 2);

                log.Info("Least Sq Done " + DateTime.Now);

                // create a dxf for those who want to "see" the calibration
                DxfDocument dxf = new DxfDocument();

                Polyline3d polyline = new Polyline3d(vertexes, true);
                polyline.Layer = new Layer("polyline3d");
                polyline.Layer.Color.Index = 24;
                dxf.AddEntity(polyline);

                Point pnt = new Point(new Vector3f(-offset.Item1, -offset.Item2, -offset.Item3));
                pnt.Layer = new Layer("old offset");
                pnt.Layer.Color.Index = 22;
                dxf.AddEntity(pnt);

                pnt = new Point(new Vector3f(-(float)x[0], -(float)x[1], -(float)x[2]));
                pnt.Layer = new Layer("new offset");
                pnt.Layer.Color.Index = 21;
                dxf.AddEntity(pnt);

                dxf.Save("magoffset.dxf", DxfVersion.AutoCad2000);

                log.Info("dxf Done " + DateTime.Now);

                Array.Resize<double>(ref x, 3);

                return x;
        }

        /// <summary>
        /// Does the least sq adjustment to find the center of the sphere
        /// </summary>
        /// <param name="data">list of x,y,z data</param>
        /// <returns>offsets</returns>
        public static double[] LeastSq(List<Tuple<float, float, float>> data)
        {
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

            log.InfoFormat("{0}", rep.terminationtype);
            log.InfoFormat("{0}", alglib.ap.format(x, 2));

            return x;
        }

        /// <summary>
        /// saves the offests to eeprom, os displays if cant
        /// </summary>
        /// <param name="ofs">offsets</param>
        public static void SaveOffsets(double[] ofs)
        {
            if (MainV2.comPort.param.ContainsKey("COMPASS_OFS_X"))
            {
                try
                {
                    // disable learning
                    MainV2.comPort.setParam("COMPASS_LEARN", 0);
                    // set values
                    MainV2.comPort.setParam("COMPASS_OFS_X", (float)ofs[0]);
                    MainV2.comPort.setParam("COMPASS_OFS_Y", (float)ofs[1]);
                    MainV2.comPort.setParam("COMPASS_OFS_Z", (float)ofs[2]);
                }
                catch { 
                    CustomMessageBox.Show("Set Compass offset failed"); 
                    return; 
                }

                CustomMessageBox.Show("New offsets are " + ofs[0].ToString("0") + " " + ofs[1].ToString("0") + " " + ofs[2].ToString("0") +"\nThese have been saved for you.", "New Mag Offsets");
            }
            else
            {
                CustomMessageBox.Show("New offsets are " + ofs[0].ToString("0") + " " + ofs[1].ToString("0") + " " + ofs[2].ToString("0") + "\n\nPlease write these down for manual entry", "New Mag Offsets");
            }
        }
        /// <summary>
        /// Min or max finder
        /// </summary>
        /// <param name="value">value to process</param>
        /// <param name="min">current min</param>
        /// <param name="max">current max</param>
        private static void setMinorMax(float value, ref float min, ref float max) 
        {
            if (value > max)         
                max = value;
            if (value < min)
                min = value;
        }


        static void sphere_error(double[] xi, double[] fi, object obj)
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