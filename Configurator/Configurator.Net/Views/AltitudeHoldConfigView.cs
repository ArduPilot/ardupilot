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
     
        }

        public override void SetDataContext(AltitudeHoldConfigVm vm)
        {
            BindButtons(vm);

            AltitudeHoldConfigBindingSource.DataSource = vm;

            if (Program.IsMonoRuntime)
                vm.PropertyChanged += ((sender, e) => AltitudeHoldConfigBindingSource.ResetBindings(false));
        }
    }

    // Required for VS2008 designer. No functional value
    public class AltitudeControlDesignable : ViewCommon<AltitudeHoldConfigVm> { }
}
