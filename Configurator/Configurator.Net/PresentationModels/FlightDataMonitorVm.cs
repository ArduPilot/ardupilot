using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;

namespace ArducopterConfigurator.PresentationModels
{
    public class FlightDataVm : MonitorVm
    {
        public FlightDataVm(CommsSession _sp) : base(_sp)
        {
          
        }

        // 2,-10,3,-2,1011,1012,1002,1000,1001,1003,1002,1004
        // Loop Time = 2
        // Roll Gyro Rate = -10
        // Pitch Gyro Rate = 3
        // Yaw Gyro Rate = -2
        // Throttle Output = 1011
        // Roll PID Output = 1012
        // Pitch PID Output = 1002
        // Yaw PID Output 1000
        // Front Motor Command = 1001 PWM output sent to right motor (ranges from 1000-2000)
        // Rear Motor Command 1003
        // Right Motor Command = 1002
        // Left Motor Command = 1004
        // then adc 4,3, and 5
        protected override void OnStringReceived(string data)
        {
            var strs = data.Split(',');
            var ints = new List<int>();
            foreach (var s in strs)
            {
                int val;
                if (!int.TryParse(s, out val))
                {
                    Debug.WriteLine("Could not parse expected integer: " + s);
                    return;
                }
                ints.Add(val);
            }

            if (ints.Count!=15)
            {
                Debug.WriteLine("Flight Data seentence expected, but only got one of size: " + ints.Count);
                return;
            }

            LoopTime = ints[0];
            GyroRoll = ints[1] + 500;
            GyroPitch = ints[2] + 500;
            GyroYaw = ints[3] + 500;
            MotorFront = ints[8];
            MotorRear = ints[9];
            MotorRight = ints[10];
            MotorLeft = ints[11];
            AccelRoll = ints[12] + 500;
            AccelPitch = ints[13] + 500;
            AccelZ = ints[14] + 500;
        }


        private int _loopTime;
        public int LoopTime
        {
            get { return _loopTime; }
            set
            {
                if (_loopTime == value) return;
                _loopTime = value;
                FirePropertyChanged("LoopTime");
            }
        }


        private int motorFront;
        public int MotorFront
        {
            get { return motorFront; }
            set {
                if (motorFront == value) return;
                motorFront = value;
                FirePropertyChanged("MotorFront");
            }
        }

        private int motorRear;
        public int MotorRear
        {
            get { return motorRear; }
            set
            {
                if (motorRear == value) return;
                motorRear = value;
                FirePropertyChanged("MotorRear");
            }
        }

        private int motorLeft;
        public int MotorLeft
        {
            get { return motorLeft; }
            set
            {
                if (motorLeft == value) return;
                motorLeft = value;
                FirePropertyChanged("MotorLeft");
            }
        }

        private int motorRight;
        public int MotorRight
        {
            get { return motorRight; }
            set
            {
                if (motorRight == value) return;
                motorRight = value;
                FirePropertyChanged("MotorRight");
            }
        }

        private int gyroPitch;
        public int GyroPitch
        {
            get { return gyroPitch; }
            set
            {
                if (gyroPitch == value) return;
                gyroPitch = value;
                FirePropertyChanged("GyroPitch");
            }
        }

        private int gyroRoll;
        public int GyroRoll
        {
            get { return gyroRoll; }
            set
            {
                if (gyroRoll == value) return;
                gyroRoll = value;
                FirePropertyChanged("GyroRoll");
            }
        }

        private int gyroYaw;
        public int GyroYaw
        {
            get { return gyroYaw; }
            set
            {
                if (gyroYaw == value) return;
                gyroYaw = value;
                FirePropertyChanged("GyroYaw");
            }
        }

        private int accelPitch;
        public int AccelPitch
        {
            get { return accelPitch; }
            set
            {
                if (accelPitch == value) return;
                accelPitch = value;
                FirePropertyChanged("AccelPitch");
            }
        }

        private int accelRoll;
        public int AccelRoll
        {
            get { return accelRoll; }
            set
            {
                if (accelRoll == value) return;
                accelRoll = value;
                FirePropertyChanged("AccelRoll");
            }
        }

        private int accelZ;
        public int AccelZ
        {
            get { return accelZ; }
            set
            {
                if (accelZ == value) return;
                accelZ = value;
                FirePropertyChanged("AccelZ");
            }
        }

        protected override void OnDeactivated()
        {
            SendString("X");
        }


        protected override void OnActivated()
        {
            SendString("S");
        }

        public override string Name
        {
            get { return "Flight Data"; }
        }
    }
}