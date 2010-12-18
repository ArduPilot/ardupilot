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

        public override void SetDataContext(CalibrationOffsetsDataVm model)
        {
            calibrationOffsetsDataVmBindingSource.DataSource = model;
        }
    }

    // Required for VS2008 designer. No functional value
    public class CalibrationViewDesignable : ViewCommon<CalibrationOffsetsDataVm> { }
}
