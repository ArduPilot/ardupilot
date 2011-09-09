using System.Collections.Generic;
using System.ComponentModel;

namespace ArducopterConfigurator.PresentationModels
{
    public class PositionAltitudePidsVm : DoubleVm<PositionHoldConfigVm,AltitudeHoldConfigVm>
    {
        public PositionAltitudePidsVm() : base(new PositionHoldConfigVm(), new AltitudeHoldConfigVm())
        {
        }

        public override string Name
        {
            get { return "Position & Altitude"; }
        }

    }
}