using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using ZedGraph;
using AGaugeApp;
using System.IO.Ports;
using System.Threading;

namespace ArdupilotMega
{
    public partial class RAW_Sensor : Form
    {
        // for graph
        RollingPointPairList list1 = new RollingPointPairList(10 * 50);
        RollingPointPairList list2 = new RollingPointPairList(10 * 50);
        RollingPointPairList list3 = new RollingPointPairList(10 * 50);
        RollingPointPairList list4 = new RollingPointPairList(10 * 50);
        RollingPointPairList list5 = new RollingPointPairList(10 * 50);
        RollingPointPairList list6 = new RollingPointPairList(10 * 50);
        object thisLock = new object();

        int tickStart = 0;

        public RAW_Sensor()
        {
            InitializeComponent();

            CreateChart(zg1, "Raw Sensors", "Time", "Raw Data");
        }

        public struct plot
        {
            public string Name;
            public RollingPointPairList PointList;
            public Color color;
        }

        public void CreateChart(ZedGraphControl zgc,string Title, string XAxis, string YAxis)
        {
            GraphPane myPane = zgc.GraphPane;

            // Set the titles and axis labels
            myPane.Title.Text = Title;
            myPane.XAxis.Title.Text = XAxis;
            myPane.YAxis.Title.Text = YAxis;

            LineItem myCurve;

            myCurve = myPane.AddCurve("Accel X", list1, Color.Red, SymbolType.None);
            myCurve = myPane.AddCurve("Accel Y", list2, Color.Green, SymbolType.None);
            myCurve = myPane.AddCurve("Accel Z", list3, Color.SandyBrown, SymbolType.None);
            myCurve = myPane.AddCurve("Gyro X", list4, Color.Blue, SymbolType.None);
            myCurve = myPane.AddCurve("Gyro Y", list5, Color.Black, SymbolType.None);
            myCurve = myPane.AddCurve("Gyro Z", list6, Color.Violet, SymbolType.None);


            // Show the x axis grid
            myPane.XAxis.MajorGrid.IsVisible = true;

            myPane.XAxis.Scale.Min = 0;
            myPane.XAxis.Scale.Max = 5;

            // Make the Y axis scale red
            myPane.YAxis.Scale.FontSpec.FontColor = Color.Red;
            myPane.YAxis.Title.FontSpec.FontColor = Color.Red;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.YAxis.MajorTic.IsOpposite = false;
            myPane.YAxis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.YAxis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.YAxis.Scale.Align = AlignP.Inside;
            // Manually set the axis range
            //myPane.YAxis.Scale.Min = -1;
            //myPane.YAxis.Scale.Max = 1;

            // Fill the axis background with a gradient
            myPane.Chart.Fill = new Fill(Color.White, Color.LightGray, 45.0f);

            // Sample at 20ms intervals
            timer1.Interval = 100;
            timer1.Enabled = true;
            timer1.Start();


            // Calculate the Axis Scale Ranges
            zgc.AxisChange();

            tickStart = Environment.TickCount;


        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            double time = (Environment.TickCount - tickStart) / 1000.0;

            // Make sure that the curvelist has at least one curve
            if (zg1.GraphPane == null || zg1.GraphPane.CurveList.Count <= 0)
                return;

            // Get the first CurveItem in the graph
            LineItem curve = zg1.GraphPane.CurveList[0] as LineItem;
            if (curve == null)
                return;

            // Get the PointPairList
            IPointListEdit list = curve.Points as IPointListEdit;
            // If this is null, it means the reference at curve.Points does not
            // support IPointListEdit, so we won't be able to modify it
            if (list == null)
                return;

            // Time is measured in seconds
            //double time = (Environment.TickCount - tickStart) / 1000.0;

            // Keep the X scale at a rolling 30 second interval, with one
            // major step between the max X value and the end of the axis
            Scale xScale = zg1.GraphPane.XAxis.Scale;
            if (time > xScale.Max - xScale.MajorStep)
            {
                xScale.Max = time + xScale.MajorStep;
                xScale.Min = xScale.Max - 10.0;
            }

            // Make sure the Y axis is rescaled to accommodate actual data
            try
            {
                zg1.AxisChange();
            }
            catch { }
            // Force a redraw
            zg1.Invalidate();
        }

        private void timer2serial_Tick(object sender, EventArgs e)
        {
            if (!MainV2.comPort.BaseStream.IsOpen && !MainV2.comPort.logreadmode)
                return;

            //Console.WriteLine(DateTime.Now.Millisecond + " timer2 serial");
            try
            {
                MainV2.comPort.MAV.cs.UpdateCurrentSettings(currentStateBindingSource);
            }
            catch { }

            if (sw != null && sw.BaseStream.CanWrite)
            {
                sw.WriteLine(string.Format("{0},{1},{2},{3},{4},{5},{6}",DateTime.Now.ToString(), MainV2.comPort.MAV.cs.ax, MainV2.comPort.MAV.cs.ay, MainV2.comPort.MAV.cs.az, MainV2.comPort.MAV.cs.gx,MainV2.comPort.MAV.cs.gy, MainV2.comPort.MAV.cs.gz));
            }

            double time = (Environment.TickCount - tickStart) / 1000.0;

            if (chkax.Checked)
            {
                list1.Add(time, ArdupilotMega.MainV2.comPort.MAV.cs.ax);
            }
            else { list1.Clear(); }
            if (chkay.Checked)
            {
                list2.Add(time, ArdupilotMega.MainV2.comPort.MAV.cs.ay);
            }
            else { list2.Clear(); }
            if (chkaz.Checked)
            {
                list3.Add(time, ArdupilotMega.MainV2.comPort.MAV.cs.az);
            }
            else { list3.Clear(); }
            if (chkgx.Checked)
            {
                list4.Add(time, ArdupilotMega.MainV2.comPort.MAV.cs.gx);
            }
            else { list4.Clear(); }
            if (chkgy.Checked)
            {
                list5.Add(time, ArdupilotMega.MainV2.comPort.MAV.cs.gy);
            }
            else { list5.Clear(); }
            if (chkgz.Checked)
            {
                list6.Add(time, ArdupilotMega.MainV2.comPort.MAV.cs.gz);
            }
            else { list6.Clear(); }
        }

        private void ACM_Setup_Load(object sender, EventArgs e)
        {

            timer2serial.Interval = 10;
            timer2serial.Enabled = true;
            timer2serial.Start();

            tabControl.SelectedTab = tabRadio;

            tabControl.SelectedTab = tabRawSensor;

            //tabControl1_SelectedIndexChanged(sender, e);


        }

        private void ACM_Setup_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (MainV2.comPort != null && MainV2.comPort.BaseStream.IsOpen)
            {
                try
                {
                    if (sw != null)
                        sw.Close();
                }
                catch { }
            }
            timer1.Stop();
            timer2serial.Stop();
        }

        private void tabControl1_SelectedIndexChanged(object sender, EventArgs e)
        {
                    try
                    {
                        if (!MainV2.comPort.BaseStream.IsOpen && !MainV2.comPort.logreadmode)
                        {
                            CustomMessageBox.Show("Please connect first");
                            this.Close();
                        }

                        //comPort.DtrEnable = true;
                        //comPort.Open();
                        //comPort.stopall(true); // ensure off

                        //comPort.requestDatastream((byte)ArdupilotMega.MAVLink09.MAV_DATA_STREAM.EXTENDED_STATUS, 0); // mode gps raw
                        //comPort.requestDatastream((byte)ArdupilotMega.MAVLink09.MAV_DATA_STREAM.POSITION, 3); // request location
                        //comPort.requestDatastream((byte)ArdupilotMega.MAVLink09.MAV_DATA_STREAM.EXTRA1, 3); // request attitude
                        //comPort.requestDatastream((byte)ArdupilotMega.MAVLink09.MAV_DATA_STREAM.EXTRA2, 3); // request vfr
                        MainV2.comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.RAW_SENSORS, MainV2.comPort.MAV.cs.ratesensors); // request raw sensor
                        //comPort.requestDatastream((byte)ArdupilotMega.MAVLink09.MAV_DATA_STREAM.RC_CHANNELS, 3); // request rc info
                    }
                    catch
                    {
                        CustomMessageBox.Show("Comport open failed");
                        return;
                    }
                timer1.Start();
        }

        private void CMB_rawupdaterate_SelectedIndexChanged(object sender, EventArgs e)
        {
            MainV2.comPort.MAV.cs.ratesensors = (byte)int.Parse(CMB_rawupdaterate.Text);
            MainV2.comPort.requestDatastream((byte)ArdupilotMega.MAVLink.MAV_DATA_STREAM.RAW_SENSORS, (byte)int.Parse(CMB_rawupdaterate.Text)); // request raw sensor
        }

        System.IO.StreamWriter sw = null;

        private void BUT_savecsv_Click(object sender, EventArgs e)
        {
            SaveFileDialog ofd = new SaveFileDialog();
            ofd.AddExtension = true;
            ofd.DefaultExt = ".csv";
            ofd.ShowDialog();
            if (ofd.FileName != "")
            {
                sw = new System.IO.StreamWriter(ofd.OpenFile());
            }
        }

    }


}
