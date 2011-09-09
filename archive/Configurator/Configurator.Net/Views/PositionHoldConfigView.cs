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
          
        }

        public override void SetDataContext(PositionHoldConfigVm vm)
        {
            BindButtons(vm);


            PositionHoldConfigBindingSource.DataSource = vm;

            if (Program.IsMonoRuntime)
                vm.PropertyChanged += ((sender, e) => PositionHoldConfigBindingSource.ResetBindings(false));
        }

        private void label8_Click(object sender, System.EventArgs e)
        {

        }
    }
    // Required for VS2008 designer. No functional value
    public class PositionHoldConfigViewDesignable : ViewCommon<PositionHoldConfigVm> { }
}
