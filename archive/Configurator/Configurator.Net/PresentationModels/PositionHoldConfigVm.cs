using System;

namespace ArducopterConfigurator.PresentationModels
{
    public class PositionHoldConfigVm : ConfigWithPidsBase
    {
        public PositionHoldConfigVm()
        {
            updateString = "C";
            refreshString = "D";

            PropsInUpdateOrder = new[] 
                                     { 
                                         "RollP", 
                                         "RollI", 
                                         "RollD", 
                                         "PitchP", 
                                         "PitchI", 
                                         "PitchD", 
                                         "MaximumAngle", 
                                         "GeoCorrectionFactor", 
                                     };
        }

        public float MaximumAngle { get;  set; }

        public float GeoCorrectionFactor { get; set; }

        public override string Name
        {
            get { return "Position Hold"; }
        }
    }
}