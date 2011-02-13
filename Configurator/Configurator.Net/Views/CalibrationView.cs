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
    public partial class CalibrationView : CalibrationViewDesignable
    {
        public CalibrationView()
        {
            InitializeComponent();
            BindButtons();
        }

        public override void SetDataContext(CalibrationOffsetsDataVm vm)
        {
            calibrationOffsetsDataVmBindingSource.DataSource = vm;


            if (Program.IsMonoRuntime)
                vm.PropertyChanged += ((sender, e) => calibrationOffsetsDataVmBindingSource.ResetBindings(false));
        }
    }

    // Required for VS2008 designer. No functional value
    public class CalibrationViewDesignable : ViewCommon<CalibrationOffsetsDataVm> { }
}
