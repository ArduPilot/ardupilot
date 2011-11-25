using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega
{
    public partial class Splash : Form
    {
        public Splash()
        {
            InitializeComponent();

            string strVersion = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString();

            TXT_version.Text = "Version: " + Application.ProductVersion;// +" Build " + strVersion;
        }
    }
}
