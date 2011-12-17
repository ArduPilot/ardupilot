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

        private void BUT_updatecheck_Click(object sender, EventArgs e)
        {
            Form loading = new Form();
            loading.Width = 400;
            loading.Height = 150;
            loading.StartPosition = FormStartPosition.CenterScreen;
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainV2));
            loading.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));

            Label loadinglabel = new Label();
            loadinglabel.Location = new System.Drawing.Point(50, 40);
            loadinglabel.Name = "load";
            loadinglabel.AutoSize = true;
            loadinglabel.Text = "Checking...";
            loadinglabel.Size = new System.Drawing.Size(100, 20);

            loading.Controls.Add(loadinglabel);
            loading.Show();

            System.Threading.Thread t12 = new System.Threading.Thread(delegate() { try { MainV2.updatecheck(loadinglabel); } catch (Exception ex) { Console.WriteLine(ex.ToString()); } }); // wait for tcp connections
            t12.Name = "Update check thread";
            t12.Start();
            MainV2.threads.Add(t12);
        }

        private void CHK_showconsole_CheckedChanged(object sender, EventArgs e)
        {
            MainV2.config["showconsole"] = CHK_showconsole.Checked.ToString();
        }

        private void Help_Load(object sender, EventArgs e)
        {
            richTextBox1.Rtf = new ComponentResourceManager(this.GetType()).GetString("Help_text");
        }
    }
}
