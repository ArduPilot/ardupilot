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
    public partial class GpsStatusView : GpsViewDesignable
    {
        public GpsStatusView()
        {
            InitializeComponent();
          
        }

        public override void SetDataContext(GpsStatusVm vm)
        {
            BindButtons(vm);

            GpsStatusBindingSource.DataSource = vm;

            if (Program.IsMonoRuntime)
                vm.PropertyChanged += ((sender, e) => GpsStatusBindingSource.ResetBindings(false));
        }

    }

      // Required for VS2008 designer. No functional value
    public class GpsViewDesignable : ViewCommon<GpsStatusVm> { }
}
