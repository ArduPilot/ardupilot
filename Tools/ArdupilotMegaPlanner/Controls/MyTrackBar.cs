using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega
{
    class MyTrackBar : TrackBar
    {
        public new double Maximum { get { return base.Maximum / 100.0; } set { base.Maximum = (int)(value * 100); } }
        public new double Minimum { get { return base.Minimum / 100.0; } set { base.Minimum = (int)(value * 100); } }
        public new double Value   { get { return base.Value / 100.0; }   set { base.Value = (int)(value * 100);   } }
    }
}
