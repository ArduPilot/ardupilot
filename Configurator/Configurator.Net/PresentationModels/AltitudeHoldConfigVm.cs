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

        public float P { get;  set; }
        public float I { get;  set; }
        public float D { get;  set; }

        public override string Name
        {
            get { return "Altitude Hold"; }
        }
    }
}