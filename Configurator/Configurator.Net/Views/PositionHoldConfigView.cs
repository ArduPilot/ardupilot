using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Text;
using ArducopterConfigurator.PresentationModels;

namespace ArducopterConfigurator.Views
{
    public partial class PositionHoldConfigView : PositionHoldConfigViewDesignable
    {
        public PositionHoldConfigView()
        {
            InitializeComponent();
            BindButtons();
        }

        public override void SetDataContext(PositionHoldConfigVm model)
        {
            PositionHoldConfigBindingSource.DataSource = model;
            _vm = model;
        }
    }
    // Required for VS2008 designer. No functional value
    public class PositionHoldConfigViewDesignable : ViewCommon<PositionHoldConfigVm> { }
}
