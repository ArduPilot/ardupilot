using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Text;
using System.Windows.Forms;
using ArducopterConfigurator.PresentationModels;

namespace ArducopterConfigurator
{
    public partial class FlightControlPidsView : UserControl, IView<FlightControlPidsVm>
    {
        private IPresentationModel _vm;

        public FlightControlPidsView()
        {
            InitializeComponent();
        }

        public void SetDataContext(FlightControlPidsVm vm)
        {
            FlightControlPidsBindingSource.DataSource = vm;
            _vm = vm;

            stableConfigView1.SetDataContext(vm.Vm2);
            acroConfigView1.SetDataContext(vm.Vm1);
        }

     
        public Control Control
        {
            get { return this; }
        }

 
    }
}
