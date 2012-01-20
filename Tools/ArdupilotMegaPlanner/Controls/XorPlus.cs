using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega
{
    public partial class XorPlus : Form
    {
        public new event EventHandler Click;

        /// <summary>
        /// either X or +
        /// </summary>
        public string frame = "";

        public XorPlus()
        {
            InitializeComponent();
        }

        private void pictureBoxQuad_Click(object sender, EventArgs e)
        {
            frame = "+";
            if (Click != null)
            {
                Click(sender, new EventArgs());
            }

            this.Close();
        }

        private void pictureBoxQuadX_Click(object sender, EventArgs e)
        {
            frame = "X";
            if (Click != null)
            {
                Click(sender, new EventArgs());
            }

            this.Close();
        }
    }
}
