using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega.Controls
{
    public partial class ModifyandSet : UserControl
    {
        [System.ComponentModel.Browsable(true)]
        public NumericUpDown NumericUpDown { get; set; }
        [System.ComponentModel.Browsable(true)]
        public MyButton Button { get; set; }

        [System.ComponentModel.Browsable(true)]
        public String ButtonText { get { return Button.Text; } set { Button.Text = value; } }
        [System.ComponentModel.Browsable(true)]
        public Decimal Value { get { return NumericUpDown.Value; } set { NumericUpDown.Value = value; } }

        public new event EventHandler Click;
        public event EventHandler ValueChanged;

        public ModifyandSet()
        {
            InitializeComponent();

            NumericUpDown = numericUpDown1;
            Button = myButton1;
        }

        private void myButton1_Click(object sender, EventArgs e)
        {
            if (Click != null)
                Click(sender, e);
        }

        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {
            if (ValueChanged != null)
                ValueChanged(sender, e);
        }
    }
}
