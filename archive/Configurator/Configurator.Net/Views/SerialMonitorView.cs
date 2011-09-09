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
    public partial class SerialMonitorView : UserControl, IView<SerialMonitorVm>
    {
        private IPresentationModel _vm;

        public SerialMonitorView()
        {
            InitializeComponent();
        }

        public void SetDataContext(SerialMonitorVm vm)
        {
            serialMonitorVmBindingSource.DataSource = vm;
            _vm = vm;
        }

     
        public Control Control
        {
            get { return this; }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            (_vm as SerialMonitorVm).SendTextCommand();
        }
    }
}
