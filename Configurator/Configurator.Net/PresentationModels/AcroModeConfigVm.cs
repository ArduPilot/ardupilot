using System;

namespace ArducopterConfigurator.PresentationModels
{
    public class AcroModeConfigVm : ConfigWithPidsBase
    {
        public AcroModeConfigVm()
        {
            updateString = "O";
            refreshString = "P"; 

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
                           "TransmitterFactor", 
                       };
        }

        public float TransmitterFactor { get; set; }

        public override string Name
        {
            get { return "Acro Mode"; }
        }
    }
}