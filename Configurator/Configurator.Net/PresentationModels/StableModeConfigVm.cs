using System;

namespace ArducopterConfigurator.PresentationModels
{
    public class StableModeConfigVm : ConfigWithPidsBase
    {
        public StableModeConfigVm()
        {
            updateString = "A";
            refreshString = "B";

            PropsInUpdateOrder = new[]
                                     {
                                         "RollP",
                                         "RollI",
                                         "RollD",
                                         "PitchP",
                                         "PitchI",
                                         "PitchD",
                                         "YawP",
                                         "YawI",
                                         "YawD",
                                         "KPrate",
                                         "MagnetometerEnable",
                                     };
        }


        private float _kprate;

        public float KPrate
        {
            get { return _kprate; }
            set
            {
                if (_kprate == value) return;
                _kprate = value;
                FirePropertyChanged("KPrate");
            }
        }


        private bool _magEnable;

        public bool MagnetometerEnable
        {
            get { return _magEnable; }
            set
            {
                if (_magEnable == value) return;
                _magEnable = value;
                FirePropertyChanged("MagnetometerEnable");
            }
        }

        public override string Name
        {
            get { return "Stable Mode"; }
        }
    }
}