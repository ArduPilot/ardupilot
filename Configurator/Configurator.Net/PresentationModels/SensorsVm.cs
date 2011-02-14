using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;

namespace ArducopterConfigurator.PresentationModels
{
    /// <summary>
    /// View Model for the sensors monitor and calibration 
    /// </summary>
    /// <remarks>
    /// This tab is for the monitoring and calibration of the Gyro/Accel sensors
    /// 
    /// When it is activated, it retrieves the current sensor calibration values
    /// Then, it requests the constant stream of sensor readings in order to provide
    /// a live view.
    /// 
    /// The user can change the calibration offsets, and upload the new values.
    /// When this happens, the model tells the APM to cease sending the realtime updates,
    /// then sends the new offset values, then resumes the updates
    /// 
    /// There is a command to automatically fill the sensor offset values, from the 
    /// current values of the sensors. The user would do this when the copter is
    /// sitting still and level 
    /// 
    /// When the VM is deactivated, the model tells the APM to cease sending the realtime updates
    /// </remarks>
    public class SensorsVm : VmBase, IPresentationModel
    {
        public string Name
        {
            get { return "Sensor Data"; }
        }

        public void Activate()
        {
            if (sendTextToApm!=null)
                sendTextToApm(this, new sendTextToApmEventArgs("S"));
                
        }

        public void DeActivate()
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs("X"));
                
        }

        public event EventHandler updatedByApm;

        
        public void handleLineOfText(string strRx)
        {
            var strs = strRx.Split(',');
            var ints = new List<int>();
            foreach (var s in strs)
            {
                int val;
                if (!int.TryParse(s, out val))
                {
                    Debug.WriteLine("(Flight Data) Could not parse expected integer: " + s);
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
            GyroRoll = ints[1];
            GyroPitch = ints[2];
            GyroYaw = ints[3];
            MotorFront = ints[8];
            MotorRear = ints[9];
            MotorRight = ints[10];
            MotorLeft = ints[11];
            AccelRoll = ints[12];
            AccelPitch = ints[13];
            AccelZ = ints[14];
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;


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
    }
}