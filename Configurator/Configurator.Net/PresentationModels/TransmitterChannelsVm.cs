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
        }

      
        private int _roll;
        public int Roll
        {
            get { return _roll; }
            set
            {
                if (_roll == value) return;
                _roll = value;
                FirePropertyChanged("Roll");
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
            }
        }

          



            private int _rollmid;
            public int RollMidValue
        {
            get { return _rollmid; }
            set
            {
                if (_rollmid == value) return;
                _rollmid = value;
                FirePropertyChanged("RollMidValue");
            }
        }

             private int _pitchMid;
            public int PitchMidValue
        {
            get { return _pitchMid; }
            set
            {
                if (_pitchMid == value) return;
                _pitchMid = value;
                FirePropertyChanged("PitchMidValue");
            }
        }

            private int _yawMid;
            public int YawMidValue
            {
                get { return _yawMid; }
                set
                {
                    if (_yawMid == value) return;
                    _yawMid = value;
                    FirePropertyChanged("YawMidValue");
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


        private void manualPropset(string strReceived)
        {
            //   PopulatePropsFromUpdate(strReceived,false);
            var strs = strReceived.Split(',');
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

            if (ints.Count != 9)
            {
                Debug.WriteLine("Tx Data sentence expected, but only got one of size: " + ints.Count);
                return;
            }

            Roll = ints[0];
            //            Pitch = ints[1];
            //            Yaw = ints[2];
            Throttle = ints[3];
            //            Mode = ints[4];
            //            Aux = ints[5];
            //            RollMidValue = ints[6];
            //            PitchMidValue = ints[7];
            //            YawMidValue = ints[8];
        
        
        }

        public override string Name
        {
            get { return "Transmitter Channels"; }
        }
    }
}