using System;

namespace ArducopterConfigurator.PresentationModels
{
    /// <summary>
    /// Vm for Altitude hold settings
    /// </summary>
    /// <remarks>
    /// Todo: this one is weird because the APM sends and receives the values
    /// in a different order
    /// There is a unit test to cover it but it will need fixing.
    /// TODO: test this 
    /// </remarks>
    public class AltitudeHoldConfigVm : CrudVm
    {
        public AltitudeHoldConfigVm()
        {
            updateString = "E";
            refreshString = "F";
            PropsInUpdateOrder = new[] {"P", "I", "D",};

        }

    


        private float _p;
        public float P
        {
            get { return _p; }
            set
            {
                if (_p == value) return;
                _p = value;
                FirePropertyChanged("P");
            }
        }

        private float _i;

        public float I
        {
            get { return _i; }
            set
            {
                if (_i == value) return;
                _i = value;
                FirePropertyChanged("I");
            }
        }

        private float _d;

        public float D
        {
            get { return _d; }
            set
            {
                if (_d == value) return;
                _d = value;
                FirePropertyChanged("D");
            }
        }


        public override string Name
        {
            get { return "Altitude Hold"; }
        }
    }
}