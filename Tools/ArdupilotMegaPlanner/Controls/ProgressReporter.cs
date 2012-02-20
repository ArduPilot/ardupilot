using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega
{
    public partial class ProgressReporter : Form
    {
        bool cancel = false;

        public ProgressReporter()
        {
            InitializeComponent();
            cancel = false;
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            cancel = true;
            this.Close();
        }

        public void updateProgressAndStatus(int progress, string status)
        {
            //Console.WriteLine(progress + " " + status);

            if (cancel)
            {
                throw new Exception("User Canceled");
            }

            if (this.IsDisposed)
            {
                return;
            }

            try
            {
                this.Invoke((MethodInvoker)delegate
            {

                lblProgressMessage.Text = status;
                if (progress == -1)
                {
                    this.progressBar1.Style = ProgressBarStyle.Marquee;
                }
                else
                {
                    this.progressBar1.Style = ProgressBarStyle.Continuous;
                    this.progressBar1.Value = progress;
                }
            });
            }
            catch { }

            System.Windows.Forms.Application.DoEvents();
        }
    }
}
