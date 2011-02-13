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
    public partial class MotorCommandsView : MotorCommandsViewDesignable
    {
        public MotorCommandsView()
        {
            InitializeComponent();
        }

        public override void SetDataContext(MotorCommandsVm vm)
        {
            
        }

      
    }
    // Required for VS2008 designer. No functional value
    public class MotorCommandsViewDesignable : ViewCommon<MotorCommandsVm> { }
}
