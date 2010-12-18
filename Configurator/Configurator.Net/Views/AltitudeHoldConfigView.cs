using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Text;
using System.Windows.Forms;
using ArducopterConfigurator.PresentationModels;

namespace ArducopterConfigurator.Views
{
    public partial class AltitudeHoldConfigView : AltitudeControlDesignable
    {
        public AltitudeHoldConfigView()
        {
            InitializeComponent();
            BindButtons();
        }

        public override void SetDataContext(AltitudeHoldConfigVm model)
        {
            AltitudeHoldConfigBindingSource.DataSource = model;
        }
    }

    // Required for VS2008 designer. No functional value
    public class AltitudeControlDesignable : ViewCommon<AltitudeHoldConfigVm> { }
}
