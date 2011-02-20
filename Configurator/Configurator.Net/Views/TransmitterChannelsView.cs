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
    public partial class TransmitterChannelsView : TransmitterChannelsViewDesignable
    {
        public TransmitterChannelsView()
        {
            InitializeComponent();
          
        }

        public override void SetDataContext(TransmitterChannelsVm vm)
        {
            BindButtons(vm);


            TransmitterChannelsBindingSource.DataSource = vm;

            if (Program.IsMonoRuntime)
                vm.PropertyChanged += ((sender, e) => TransmitterChannelsBindingSource.ResetBindings(false));
        }

    }

      // Required for VS2008 designer. No functional value
    public class TransmitterChannelsViewDesignable : ViewCommon<TransmitterChannelsVm> { }
}
