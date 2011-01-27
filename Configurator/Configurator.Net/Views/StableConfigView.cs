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
    public partial class StableConfigView : StableConfigViewDesignable
    {
        public StableConfigView()
        {
            InitializeComponent();
            BindButtons();
        }

        public override void SetDataContext(StableModeConfigVm model)
        {
            StableModeConfigVmBindingSource.DataSource = model;

            if (Program.IsMonoRuntime)
                model.PropertyChanged += ((sender, e) => StableModeConfigVmBindingSource.ResetBindings(false));
        }

    }

    // Required for VS2008 designer. No functional value
    public class StableConfigViewDesignable : ViewCommon<StableModeConfigVm> { }
}
