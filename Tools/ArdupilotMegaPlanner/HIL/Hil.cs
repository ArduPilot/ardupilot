using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using log4net;
using System.Reflection;
using System.Runtime.InteropServices;

namespace ArdupilotMega.HIL
{
    public delegate void ProgressEventHandler(int progress, string status);

    public abstract class Hil
    {
        internal static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct sitl_fdm
        {
            // this is the packet sent by the simulator
            // to the APM executable to update the simulator state
            // All values are little-endian
            public double latitude, longitude; // degrees
            public double altitude;  // MSL
            public double heading;   // degrees
            public double speedN, speedE; // m/s
            public double xAccel, yAccel, zAccel;       // m/s/s in body frame
            public double rollRate, pitchRate, yawRate; // degrees/s/s in earth frame
            public double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
            public double airspeed; // m/s
            public UInt32 magic; // 0x4c56414e
        };

        public const float ft2m = (float)(1.0 / 3.2808399);
        public const float rad2deg = (float)(180 / Math.PI);
        public const float deg2rad = (float)(1.0 / rad2deg);
        public const float kts2fps = (float)1.68780986;

        internal DateTime lastgpsupdate = DateTime.Now;
        internal sitl_fdm[] sitl_fdmbuffer = new sitl_fdm[5];
        internal sitl_fdm oldgps = new sitl_fdm();

        // gps buffer
        internal int gpsbufferindex = 0;

        internal int REV_pitch = 1;
        internal int REV_roll = 1;
        internal int REV_rudder = 1;
        internal int GPS_rate = 200;

        internal int rollgain = 10000;
        internal int pitchgain = 10000;
        internal int ruddergain = 10000;
        internal int throttlegain = 10000;

        public float roll_out, pitch_out, throttle_out, rudder_out, collective_out;

        public event ProgressEventHandler Status;

        public bool sitl { get; set; }
        public bool heli { get; set; }
        public bool quad { get; set; }
        public bool xplane10 { get; set; }

        public abstract void SetupSockets(int Recvport, int SendPort, string SimIP);
        public abstract void Shutdown();

        public abstract void GetFromSim(ref sitl_fdm data);
        public abstract void SendToSim();
        public abstract void SendToAP(sitl_fdm data);
        public abstract void GetFromAP();

        internal void UpdateStatus(int progress, string status)
        {
            if (Status != null)
                Status(progress, status);
        }

        internal float Constrain(float value, float min, float max)
        {
            if (value > max) { value = max; }
            if (value < min) { value = min; }
            return value;
        }

        internal short Constrain(double value, double min, double max)
        {
            if (value > max) { value = max; }
            if (value < min) { value = min; }
            return (short)value;
        }
    }
}
