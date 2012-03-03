using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega.GCSViews
{
    public partial class Help : MyUserControl
    {
        public Help()
        {
            InitializeComponent();

            try
            {
                CHK_showconsole.Checked = MainV2.config["showconsole"].ToString() == "True";
            }
            catch { }
        }

        public void BUT_updatecheck_Click(object sender, EventArgs e)
        {
            MainV2.DoUpdate();
        }

        private void CHK_showconsole_CheckedChanged(object sender, EventArgs e)
        {
            MainV2.config["showconsole"] = CHK_showconsole.Checked.ToString();
        }

        private void Help_Load(object sender, EventArgs e)
        {
            richTextBox1.Rtf = new ComponentResourceManager(this.GetType()).GetString("help_text");
        }
    }
}
