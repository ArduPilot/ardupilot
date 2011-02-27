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
    public class SensorsVm : NotifyProperyChangedBase, IPresentationModel
    {

        private const string CALIB_REFRESH = "J";
        private const string CALIB_UPDATE = "I";
        private const string SEND_SENSOR_DATA = "S";
        private const string STOP_UPDATES = "X";

        private bool waitingForCalibData;

        public SensorsVm()
        {
            RefreshCalibrationOffsetsCommand = new DelegateCommand(_ => RefreshCalibValues());
            UpdateCalibrationOffsetsCommand = new DelegateCommand(_ => UpdateCalibValues());
            CalculateCalibrationOffsetsCommand = new DelegateCommand(_ => CalcCalibValues());

            PropertyChanged += ((sender, e) => 
            {                        
                IsArmed = !(MotorFront == 1040 && MotorRear == 1040 && MotorLeft == 1040 && MotorRight == 1040);
            });
        }

        public void RefreshCalibValues()
        {
            sendString(STOP_UPDATES);
            waitingForCalibData = true;
            sendString(CALIB_REFRESH);
        }

        public void UpdateCalibValues()
        {
            sendString(PropertyHelper.ComposePropValuesWithCommand(this, _calibrationPropsInUpdateOrder, CALIB_UPDATE));
            sendString(SEND_SENSOR_DATA);
        }

        public void CalcCalibValues()
        {
            AccelRollOffset = AccelRollOffset - AccelRoll;
            AccelPitchOffset = AccelPitchOffset - AccelPitch;
            //AccelZOffset = AccelZOffset - AccelZ;
            FirePropertyChanged("AccelRollOffset");
            FirePropertyChanged("AccelPitchOffset");
            //FirePropertyChanged("AccelZOffset");
        }

        public string Name
        {
            get { return "Sensor Data"; }
        }

      


        private readonly string[] _sensorPropsInUpdateOrder = new[] 
               { 
                   "LoopTime", 
                   "GyroRoll", 
                   "GyroPitch", 
                   "GyroYaw", 
                   "Unused", // Throttle
                   "ControlRoll", // control roll
                   "ControlPitch", // control pitch
                   "ControlYaw", // control yaw
                   "MotorFront", 
                   "MotorRear", 
                   "MotorRight", 
                   "MotorLeft", 
                   "AccelRoll", 
                   "AccelPitch", 
                   "AccelZ",
                   "CompassHeading", // AP_Compass.heading
                   "UnusedFloat", // AP_Compass.heading_x
                   "UnusedFloat", // AP_Compass.heading_y
                   "Unused", // AP_Compass.mag_x
                   "Unused", // AP_Compass.mag_y
                   "Unused", // AP_Compass.mag_z
               
               };

        private readonly string[] _calibrationPropsInUpdateOrder = new[] 
            { 
                "GyroRollOffset", 
                "GyroPitchOffset", 
                "GyroYawOffset", 
                "AccelRollOffset", 
                "AccelPitchOffset", 
                "AccelZOffset" 
            };
        

        private void sendString(string str)
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs(str));
        }

        public void Activate()
        {
            // Get the calib. data first
            waitingForCalibData = true;
            sendString(CALIB_REFRESH);
        }

        public void DeActivate()
        {
            sendString(STOP_UPDATES);
        }

        public void handleLineOfText(string strRx)
        {
            if ( waitingForCalibData)
            {
                PropertyHelper.PopulatePropsFromUpdate(this, _calibrationPropsInUpdateOrder, strRx, true);
                waitingForCalibData = false;
                sendString(SEND_SENSOR_DATA);
            }
            else
            {
                PropertyHelper.PopulatePropsFromUpdate(this, _sensorPropsInUpdateOrder, strRx, false);
            }
        }

        public event EventHandler updatedByApm;

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;

        public ICommand RefreshCalibrationOffsetsCommand { get; private set; }

        public ICommand UpdateCalibrationOffsetsCommand { get; private set; }
        
        public ICommand CalculateCalibrationOffsetsCommand { get; private set; }


        #region Calibration Properties

        public float GyroRollOffset { get; set; }
        public float GyroPitchOffset { get; set; }
        public float GyroYawOffset { get; set; }

        public float AccelRollOffset { get; set; }
        public float AccelPitchOffset { get; set; }
        public float AccelZOffset { get; set; }


        #endregion


        private bool _isArmed;


        /// <summary>
        /// Whether the Arducopter is Armed or not
        /// </summary>
        /// <remarks>
        /// We don't get this information directly, but we can infer it if all motors are
        /// at 1040 then the thing is NOT armed.
        /// </remarks>
        public bool IsArmed
        {
            get { return _isArmed; }
            set
            {
                if (_isArmed == value) return;
                _isArmed = value;
                FirePropertyChanged("IsArmed");
            }
        }


        #region Sensor Properties

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

        private int _controlRoll;

        public int ControlRoll
        {
            get { return _controlRoll; }
            set
            {
                if (_controlRoll == value) return;
                _controlRoll = value;
                FirePropertyChanged("ControlRoll");
            }
        }

        private int _controlPitch;

        public int ControlPitch
        {
            get { return _controlPitch; }
            set
            {
                if (_controlPitch == value) return;
                _controlPitch = value;
                FirePropertyChanged("ControlPitch");
            }
        }

        private int _controlYaw;

        public int ControlYaw
        {
            get { return _controlYaw; }
            set
            {
                if (_controlYaw == value) return;
                _controlYaw = value;
                FirePropertyChanged("ControlYaw");
            }
        }

        private float _compassHeading;


        /// <summary>
        /// The current compass Heading as read from the magnetometer in radians?
        /// </summary>
        public float CompassHeading
        {
            get { return _compassHeading; }
            set
            {
                if (_compassHeading == value) return;
                _compassHeading = value;
                FirePropertyChanged("CompassHeading");
                FirePropertyChanged("CompassHeadingDegrees");
            }
        }

        /// <summary>
        /// The current compass Heading as read from the magnetometer in degrees
        /// </summary>
        public float CompassHeadingDegrees
        {
            get
            {
                var degrees = (float) ((CompassHeading) * (180 / Math.PI));
                return (360 + degrees) % 360;
            }
        }

        #endregion

        public int Unused { get; set; }
        public float UnusedFloat { get; set; }
    }
}