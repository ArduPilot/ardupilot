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

        public float KPrate { get;  set; }

        public bool MagnetometerEnable { get; set; }

        public override string Name
        {
            get { return "Stable Mode"; }
        }
    }
}