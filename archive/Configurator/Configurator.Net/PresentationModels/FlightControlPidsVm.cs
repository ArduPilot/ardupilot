using System;
using System.Collections.Generic;
using System.ComponentModel;

namespace ArducopterConfigurator.PresentationModels
{
    public class FlightControlPidsVm : DoubleVm<AcroModeConfigVm, StableModeConfigVm>
    {
        public FlightControlPidsVm() : base(new AcroModeConfigVm(), new StableModeConfigVm())
        {
          
        }

        public override string Name
        {
            get { return "Flight Control"; }
        }
    }
}