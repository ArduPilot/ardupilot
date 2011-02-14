using System.Collections.Generic;
using System.Diagnostics;

namespace ArducopterConfigurator.PresentationModels
{
    public abstract class ConfigWithPidsBase :  CrudVm
    {

        public float RollP { get;  set; }
        public float RollI { get;  set; }
        public float RollD { get;  set; }

        public float PitchP { get;  set; }
        public float PitchI { get;  set; }
        public float PitchD { get;  set; }

        public float YawP { get;  set; }
        public float YawI { get;  set; }
        public float YawD { get;  set; }

     
    }
}