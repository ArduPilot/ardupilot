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
    public partial class QuickView : UserControl
    {
        [System.ComponentModel.Browsable(true)]
        public string desc { get { return labelWithPseudoOpacity1.Text; } set { labelWithPseudoOpacity1.Text = value; } }
        [System.ComponentModel.Browsable(true)]
        public string number { get { return labelWithPseudoOpacity2.Text; } set { labelWithPseudoOpacity2.Text = value; } }
        [System.ComponentModel.Browsable(true)]
        public Color numberColor { get { return labelWithPseudoOpacity2.ForeColor; } set { labelWithPseudoOpacity2.ForeColor = value; } }

        public QuickView()
        {
            InitializeComponent();

            labelWithPseudoOpacity1.DoubleClick += new EventHandler(labelWithPseudoOpacity1_DoubleClick);
            labelWithPseudoOpacity2.DoubleClick += new EventHandler(labelWithPseudoOpacity2_DoubleClick);
        }

        void labelWithPseudoOpacity2_DoubleClick(object sender, EventArgs e)
        {
            this.OnDoubleClick(e);
        }

        void labelWithPseudoOpacity1_DoubleClick(object sender, EventArgs e)
        {
            this.OnDoubleClick(e);
        }

    }
}
