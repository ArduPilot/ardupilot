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
    public partial class PositionAltitudePidsView : UserControl, IView<PositionAltitudePidsVm>
    {
        private IPresentationModel _vm;

        public PositionAltitudePidsView()
        {
            InitializeComponent();
        }

        public void SetDataContext(PositionAltitudePidsVm vm)
        {
            PositionAltitudePidsBindingSource.DataSource = vm;
            _vm = vm;

            positionHoldConfigView1.SetDataContext(vm.Vm1);
            altitudeHoldConfigView1.SetDataContext(vm.Vm2);
        }

     
        public Control Control
        {
            get { return this; }
        }
    }
}
