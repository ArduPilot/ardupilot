using System.Collections.Generic;
using System.Diagnostics;

namespace ArducopterConfigurator.PresentationModels
{
    public abstract class ConfigWithPidsBase :  CrudVm
    {

        private float _rollP;

        public float RollP
        {
            get { return _rollP; }
            set
            {
                if (_rollP == value) return;
                _rollP = value;
                FirePropertyChanged("RollP");
            }
        }

        private float _rolli;

        public float RollI
        {
            get { return _rolli; }
            set
            {
                if (_rolli == value) return;
                _rolli = value;
                FirePropertyChanged("RollI");
            }
        }

        private float _rollD;

        public float RollD
        {
            get { return _rollD; }
            set
            {
                if (_rollD == value) return;
                _rollD = value;
                FirePropertyChanged("RollD");
            }
        }

        private float _pitchP;

        public float PitchP
        {
            get { return _pitchP; }
            set
            {
                if (_pitchP == value) return;
                _pitchP = value;
                FirePropertyChanged("PitchP");
            }
        }

        private float _pitchI;

        public float PitchI
        {
            get { return _pitchI; }
            set
            {
                if (_pitchI == value) return;
                _pitchI = value;
                FirePropertyChanged("PitchI");
            }
        }

        private float _pitchD;

        public float PitchD
        {
            get { return _pitchD; }
            set
            {
                if (_pitchD == value) return;
                _pitchD = value;
                FirePropertyChanged("PitchD");
            }
        }


        private float _yawP;

        public float YawP
        {
            get { return _yawP; }
            set
            {
                if (_yawP == value) return;
                _yawP = value;
                FirePropertyChanged("YawP");
            }
        }

        private float _yawI;

        public float YawI
        {
            get { return _yawI; }
            set
            {
                if (_yawI == value) return;
                _yawI = value;
                FirePropertyChanged("YawI");
            }
        }

        private float _yawD;

        public float YawD
        {
            get { return _yawD; }
            set
            {
                if (_yawD == value) return;
                _yawD = value;
                FirePropertyChanged("YawD");
            }
        }

     
    }
}