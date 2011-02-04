using System.Collections.Generic;
using System.Diagnostics;

namespace ArducopterConfigurator.PresentationModels
{
    public class TransmitterChannelsVm : MonitorVm
    {
        public TransmitterChannelsVm(IComms sp) : base(sp)
        {
            PropsInUpdateOrder = new[] 
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

            ResetCommand = new DelegateCommand(_ => ResetWatermarks());
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


        public ICommand ResetCommand { get; private set; }

        private int _roll;
        public int Roll
        {
            get { return _roll; }
            set
            {
                if (_roll == value) return;
                _roll = value;
                FirePropertyChanged("Roll");
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


        protected override void OnActivated()
        {
            SendString("U");
        }

        protected override void OnDeactivated()
        {
            SendString("X");
        }

        protected override void OnStringReceived(string strReceived)
        {
            PopulatePropsFromUpdate(strReceived,false);
        }

        public override string Name
        {
            get { return "Transmitter Channels"; }
        }
    }
}