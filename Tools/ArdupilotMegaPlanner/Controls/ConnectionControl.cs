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
    public partial class ConnectionControl : UserControl
    {
        public ConnectionControl()
        {
            InitializeComponent();
            this.linkLabel1.Click += (sender, e) =>
                                         {
                                             if (ShowLinkStats!=null)
                                                 ShowLinkStats.Invoke(this, EventArgs.Empty);
                                         };
        }

        public event EventHandler ShowLinkStats;
        public ComboBox CMB_baudrate { get { return this.cmb_Baud; } }
        public ComboBox CMB_serialport { get { return this.cmb_Connection; } }
        public ComboBox TOOL_APMFirmware { get { return this.cmb_ConnectionType; } }

        /// <summary>
        /// Called from the main form - set whether we are connected or not currently.
        /// UI will be updated accordingly
        /// </summary>
        /// <param name="isConnected">Whether we are connected</param>
        public void IsConnected(bool isConnected)
        {
            this.linkLabel1.Visible = isConnected;
            cmb_Baud.Enabled = !isConnected;
            cmb_Connection.Enabled = !isConnected;
        }

        private void ConnectionControl_MouseClick(object sender, MouseEventArgs e)
        {
            if (e.X > cmb_ConnectionType.Location.X &&
                e.Y > cmb_ConnectionType.Location.Y &&
                e.X < cmb_ConnectionType.Right &&
                e.Y < cmb_ConnectionType.Bottom)
            {
                cmb_ConnectionType.Visible = true;
            }
        }

        private void cmb_Connection_DrawItem(object sender, DrawItemEventArgs e)
        {
            if (e.Index < 0)
                return;

            ComboBox combo = sender as ComboBox;
            if ((e.State & DrawItemState.Selected) == DrawItemState.Selected)
                e.Graphics.FillRectangle(new SolidBrush(SystemColors.Highlight),
                                         e.Bounds);
            else
                e.Graphics.FillRectangle(new SolidBrush(combo.BackColor),
                                         e.Bounds);

            string text = combo.Items[e.Index].ToString();
            if (!MainV2.MONO)
            {
                text = text + " "+ ArdupilotMega.Comms.SerialPort.GetNiceName(text);
            }

            e.Graphics.DrawString(text, e.Font,
                                  new SolidBrush(combo.ForeColor),
                                  new Point(e.Bounds.X, e.Bounds.Y));

            e.DrawFocusRectangle();
        }

    }
}
