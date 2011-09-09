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
    public partial class AcroConfigView : AcroControlDesignable
    {
        public AcroConfigView()
        {
            InitializeComponent();
        }

        public override void SetDataContext(AcroModeConfigVm vm)
        {
            BindButtons(vm);

            AcroModeConfigVmBindingSource.DataSource = vm;

            if (Program.IsMonoRuntime)
                vm.PropertyChanged += ((sender, e) => AcroModeConfigVmBindingSource.ResetBindings(false));
        }

    }

    // Required for VS2008 designer. No functional value
    public class AcroControlDesignable : ViewCommon<AcroModeConfigVm> { }
}
