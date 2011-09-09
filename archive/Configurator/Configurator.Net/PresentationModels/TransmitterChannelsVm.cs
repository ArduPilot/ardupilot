using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace ArducopterConfigurator.PresentationModels
{
    public class TransmitterChannelsVm : NotifyProperyChangedBase, IPresentationModel
    {
        private const string CALIB_REFRESH = "J";
        private const string CALIB_UPDATE = "I";
        private const string STOP_UPDATES = "X";
        private const string START_UPDATES = "U";
        private const string WRITE_TO_EEPROM = "W";

        private bool _isCalibrating;

        public bool IsCalibrating
        {
            get { return _isCalibrating; }
            set
            {
                _isCalibrating = value;
                FirePropertyChanged("IsCalibrating");
            }
        }


        private readonly string[] _propsInUpdateOrder = new[] 
            { 
                "Roll", // Aileron
                "Pitch",  // Elevator
                "Yaw", 
                "Throttle", 
                "Mode", // AUX1 (Mode)
                "Aux",  // AUX2 
                "RollMidValue", 
                "PitchMidValue",  
                "YawMidValue", 
            };


        public TransmitterChannelsVm() 
        {
            StartCalibrationCommand = new DelegateCommand(_ => BeginCalibration(),_=>!IsCalibrating);
            SaveCalibrationCommand = new DelegateCommand(_ => SaveCalibration(),_=> IsCalibrating);
            CancelCalibrationCommand = new DelegateCommand(_ => CancelCalibration(), _ => IsCalibrating);
        }


        private void sendString(string str)
        {
            if (sendTextToApm != null)
                sendTextToApm(this, new sendTextToApmEventArgs(str));
        }

        private void BeginCalibration()
        {
            // send x command to stop jabber
            sendString(STOP_UPDATES);

            ResetWatermarks();

            IsCalibrating = true;

            // send command to clear the slope and offsets
            // 11;0;1;0;1;0;1;0;1;0;1;0;
            sendString("11;0;1;0;1;0;1;0;1;0;1;0;");

            // continue the sensor
            sendString(START_UPDATES);
            
        }


        private void CancelCalibration()
        {
            IsCalibrating = false;
            HideWatermarks();
        }

        private float GetScale(int min, int max)
        {
            var span = max - min;
            return 1000F / span;
        }

        private int GetOffset(int min, int max)
        {
            return 0 - (int) ((min  * GetScale(min,max)-1000));
        }

        private void SaveCalibration()
        {
            // send values
            // eg 1.17,-291.91,1.18,-271.76,1.19,-313.91,1.18,-293.63,1.66,-1009.9

//              ch_roll_slope = readFloatSerial();
//      ch_roll_offset = readFloatSerial();
//      ch_pitch_slope = readFloatSerial();
//      ch_pitch_offset = readFloatSerial();
//      ch_yaw_slope = readFloatSerial();
//      ch_yaw_offset = readFloatSerial();
//      ch_throttle_slope = readFloatSerial();
//      ch_throttle_offset = readFloatSerial();
//      ch_aux_slope = readFloatSerial();
//      ch_aux_offset = readFloatSerial();
//      ch_aux2_slope = readFloatSerial();
//      ch_aux2_offset = readFloatSerial();

            // From Configurator:
            // 1.20,-331.74,1.20,-296.87,1.21,-350.73,1.19,-315.41,1.76,-1186.95,1.77,-1194.35



            var vals = string.Format("{0:0.00};{1:0.00};{2:0.00};{3:0.00};{4:0.00};{5:0.00};{6:0.00};{7:0.00};{8:0.00};{9:0.00};{10:0.00};{11:0.00};",
                          GetScale(RollMin, RollMax),
                          GetOffset(RollMin, RollMax),
                          GetScale(PitchMin, PitchMax),
                          GetOffset(PitchMin, PitchMax),
                          GetScale(YawMin, YawMax),
                          GetOffset(YawMin, YawMax),
                          GetScale(ThrottleMin, ThrottleMax),
                          GetOffset(ThrottleMin, ThrottleMax),
                          GetScale(AuxMin, AuxMax),                          
                          GetOffset(AuxMin, AuxMax),
                          GetScale(ModeMin, ModeMax),
                          GetOffset(ModeMin, ModeMax) // this correct?
                );


            sendString("1" + vals);

            IsCalibrating = false;
            HideWatermarks();

            // save
            //sendString(WRITE_TO_EEPROM);
        }

        private void ResetWatermarks()
        {
            ThrottleMin = ThrottleMax = Throttle;
            RollMax = RollMin = Roll;
            YawMax = YawMin = Yaw;
            PitchMax = PitchMin = Pitch;
            AuxMax = AuxMin = Aux;
            ModeMax = ModeMin = Mode;
        }

        private void HideWatermarks()
        {
            ThrottleMin = ThrottleMax = 0;
            RollMax = RollMin = 0;
            YawMax = YawMin = 0;
            PitchMax = PitchMin = 0;
            AuxMax = AuxMin = 0;
            ModeMax = ModeMin = 0;
        }

        public ICommand StartCalibrationCommand { get; private set; }
        public ICommand SaveCalibrationCommand { get; private set; }
        public ICommand CancelCalibrationCommand { get; private set; }

        public int RollMidValue { get; set; }
        public int PitchMidValue { get; set; }
        public int YawMidValue { get; set; }



        public string Name
        {
            get { return "Transmitter Channels"; }
        }

        public void Activate()
        {
            IsCalibrating = false;
            HideWatermarks();
            sendString(START_UPDATES);
        }

        public void DeActivate()
        {
            sendString(STOP_UPDATES);
        }

        public event EventHandler updatedByApm;

        public void handleLineOfText(string strRx)
        {
            PropertyHelper.PopulatePropsFromUpdate(this, _propsInUpdateOrder, strRx, false);
        }

        public event EventHandler<sendTextToApmEventArgs> sendTextToApm;

        #region bindables

        private int _roll;
        public int Roll
        {
            get { return _roll; }
            set
            {
                if (_roll == value) return;
                _roll = value;
                FirePropertyChanged("Roll");
                if (!_isCalibrating) return;
                
                if (value > RollMax)
                    RollMax = value;
                if (value < RollMin)
                    RollMin = value;
            }
        }

        private int _rollMax;
        public int RollMax
        {
            get { return _rollMax; }
            set
            {
                if (_rollMax == value) return;
                _rollMax = value;
                FirePropertyChanged("RollMax");
            }
        }

        private int _rollMin;
        public int RollMin
        {
            get { return _rollMin; }
            set
            {
                if (_rollMin == value) return;
                _rollMin = value;
                FirePropertyChanged("RollMin");
            }
        }

        private int _pitch;
        public int Pitch
        {
            get { return _pitch; }
            set
            {
                if (_pitch == value) return;
                _pitch = value;
                FirePropertyChanged("Pitch");
                if (!_isCalibrating) return;

                if (value > PitchMax)
                    PitchMax = value;
                if (value < PitchMin)
                    PitchMin = value;
            }
        }

        private int _pitchMax;
        public int PitchMax
        {
            get { return _pitchMax; }
            set
            {
                if (_pitchMax == value) return;
                _pitchMax = value;
                FirePropertyChanged("PitchMax");
            }
        }

        private int _pitchMin;
        public int PitchMin
        {
            get { return _pitchMin; }
            set
            {
                if (_pitchMin == value) return;
                _pitchMin = value;
                FirePropertyChanged("PitchMin");
            }
        }


        private int _yaw;
        public int Yaw
        {
            get { return _yaw; }
            set
            {
                if (_yaw == value) return;
                _yaw = value;
                FirePropertyChanged("Yaw");
                if (!_isCalibrating) return;
                
                if (value > YawMax)
                    YawMax = value;
                if (value < YawMin)
                    YawMin = value;
            }
        }

        private int _yawMax;
        public int YawMax
        {
            get { return _yawMax; }
            set
            {
                if (_yawMax == value) return;
                _yawMax = value;
                FirePropertyChanged("YawMax");
            }
        }

        private int _yawMin;
        public int YawMin
        {
            get { return _yawMin; }
            set
            {
                if (_yawMin == value) return;
                _yawMin = value;
                FirePropertyChanged("YawMin");
            }
        }

        private int _throttle;
        public int Throttle
        {
            get { return _throttle; }
            set
            {
                if (_throttle == value) return;
                _throttle = value;
                FirePropertyChanged("Throttle");
                if (!_isCalibrating) return;
                if (value > ThrottleMax)
                    ThrottleMax = value;
                if (value < ThrottleMin)
                    ThrottleMin = value;
            }
        }

        private int _throttleMin;
        public int ThrottleMin
        {
            get { return _throttleMin; }
            set
            {
                if (_throttleMin == value) return;
                _throttleMin = value;
                FirePropertyChanged("ThrottleMin");
            }
        }

        private int _throttleMax;
        public int ThrottleMax
        {
            get { return _throttleMax; }
            set
            {
                if (_throttleMax == value) return;
                _throttleMax = value;
                FirePropertyChanged("ThrottleMax");
            }
        }

        private int _mode;
        public int Mode
        {
            get { return _mode; }
            set
            {
                if (_mode == value) return;
                _mode = value;
                FirePropertyChanged("Mode");
                if (!_isCalibrating) return;                
                if (value > ModeMax)
                    ModeMax = value;
                if (value < ModeMin)
                    ModeMin = value;
            }
        }

        private int _modeMax;
        public int ModeMax
        {
            get { return _modeMax; }
            set
            {
                if (_modeMax == value) return;
                _modeMax = value;
                FirePropertyChanged("ModeMax");
            }
        }

        private int _modeMin;
        public int ModeMin
        {
            get { return _modeMin; }
            set
            {
                if (_modeMin == value) return;
                _modeMin = value;
                FirePropertyChanged("ModeMin");
            }
        }
        
        private int _aux;
        public int Aux
        {
            get { return _aux; }
            set
            {
                if (_aux == value) return;
                _aux = value;
                FirePropertyChanged("Aux");
                if (!_isCalibrating) return;

                if (value > AuxMax)
                    AuxMax = value;
                if (value < AuxMin)
                    AuxMin = value;
            }
        }

        private int _auxMax;
        public int AuxMax
        {
            get { return _auxMax; }
            set
            {
                if (_auxMax == value) return;
                _auxMax = value;
                FirePropertyChanged("AuxMax");
            }
        }

        private int _auxMin;

        public int AuxMin
        {
            get { return _auxMin; }
            set
            {
                if (_auxMin == value) return;
                _auxMin = value;
                FirePropertyChanged("AuxMin");
            }
        }

        #endregion


    }
}