using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ArdupilotMega.Antenna
{
    class ArduTracker : ITrackerOutput
    {
        public Comms.SerialPort ComPort { get; set; }
        /// <summary>
        ///  0-360
        /// </summary>
        public double TrimPan { get; set; }
        /// <summary>
        /// -90 - 90
        /// </summary>
        public double TrimTilt { get; set; }

        public int PanStartRange { get; set; }
        public int TiltStartRange { get; set; }
        public int PanEndRange { get; set; }
        public int TiltEndRange { get; set; }
        public int PanPWMRange { get; set; }
        public int TiltPWMRange { get; set; }
        public int PanPWMCenter { get; set; }
        public int TiltPWMCenter { get; set; }

        public bool PanReverse { get { return _panreverse == 1; } set { _panreverse = value == true ? -1 : 1; } }
        public bool TiltReverse { get { return _tiltreverse == 1; } set { _tiltreverse = value == true ? -1 : 1; } }

        int _panreverse = 1;
        int _tiltreverse = 1;

        int currentpan = 1500;
        int currenttilt = 1500;

        public bool Init()
        {

            if ((PanStartRange - PanEndRange) == 0)
            {
                System.Windows.Forms.CustomMessageBox.Show("Invalid Pan Range", "Error");
                return false;
            }

            if ((TiltStartRange - TiltEndRange) == 0)
            {
                System.Windows.Forms.CustomMessageBox.Show("Invalid Tilt Range", "Error");
                return false;
            }

            try
            {
                ComPort.Open();
            }
            catch (Exception ex) { System.Windows.Forms.CustomMessageBox.Show("Connect failed " + ex.Message, "Error"); return false; }

            return true;
        }
        public bool Setup()
        {


            return true;
        }

        double wrap_180(double input)
        {
            if (input > 180)
                return input - 360;
            if (input < -180)
                return input + 360;
            return input;
        }

        double wrap_range(double input, double range)
        {
            if (input > range)
                return input - 360;
            if (input < -range)
                return input + 360;
            return input;
        }

        public bool Pan(double Angle)
        {
            double range = Math.Abs(PanStartRange - PanEndRange);

            // get relative center based on tracking center
            double rangeleft = PanStartRange - TrimPan;
            double rangeright = PanEndRange - TrimPan;
            double centerpos = PanPWMCenter;

            // get the output angle the tracker needs to point and constrain the output to the allowed options
            short PointAtAngle = Constrain(wrap_180(Angle - TrimPan), PanStartRange, PanEndRange);

            // conver the angle into a 0-pwmrange value
            int target = (int)((((PointAtAngle / range) * 2.0) * (PanPWMRange / 2) * _panreverse + centerpos));

            // Console.WriteLine("P " + Angle + " " + target + " " + PointAtAngle);

            currentpan = target;

            return false;
        }

        public bool Tilt(double Angle)
        {
            double range = Math.Abs(TiltStartRange - TiltEndRange);

            short PointAtAngle = Constrain((Angle - TrimTilt), TiltStartRange, TiltEndRange);

            int target = (int)((((PointAtAngle / range) * 2.0) * (TiltPWMRange / 2) * _tiltreverse + TiltPWMCenter));

            // Console.WriteLine("T " + Angle + " " + target + " " + PointAtAngle);

            currenttilt = target;

            return false;
        }

        public bool PanAndTilt(double pan, double tilt)
        {
            Tilt(tilt);
            Pan(pan);

            string command = string.Format("!!!PAN:{0:0000},TLT:{1:0000}\n", currentpan, currenttilt);

            Console.Write(command);

            ComPort.Write(command);

            return false;
        }

        public bool Close()
        {
            try
            {
                ComPort.Close();
            }
            catch { }
            return true;
        }

        short Constrain(double input, double min, double max)
        {
            if (input < min)
                return (short)min;
            if (input > max)
                return (short)max;
            return (short)input;
        }

    }
}