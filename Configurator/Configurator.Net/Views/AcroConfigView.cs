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
            BindButtons();
        }

        public override void SetDataContext(AcroModeConfigVm model)
        {
            AcroModeConfigVmBindingSource.DataSource = model;

            if (Program.IsMonoRuntime)
                model.PropertyChanged += ((sender, e) => AcroModeConfigVmBindingSource.ResetBindings(false));
        }

    }

    // Required for VS2008 designer. No functional value
    public class AcroControlDesignable : ViewCommon<AcroModeConfigVm> { }
}
