using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ZedGraph;
using ArdupilotMega;
using System.Text.RegularExpressions;
using System.IO;

namespace _3DRRadio
{
    public partial class Rssi : UserControl
    {
        _3DRradio inter = new _3DRradio();
        int tickStart = 0;

        RollingPointPairList plotdatarssil = new RollingPointPairList(1200);
        RollingPointPairList plotdatarssir = new RollingPointPairList(1200);
        RollingPointPairList plotdatanoicel = new RollingPointPairList(1200);
        RollingPointPairList plotdatanoicer = new RollingPointPairList(1200);

        public Rssi()
        {
            InitializeComponent();

            zedGraphControl1.GraphPane.AddCurve("RSSI Local",plotdatarssil,Color.Red,SymbolType.None);
            zedGraphControl1.GraphPane.AddCurve("RSSI Remote", plotdatarssir, Color.Green, SymbolType.None);
            zedGraphControl1.GraphPane.AddCurve("Noise Local", plotdatanoicel, Color.Blue, SymbolType.None);
            zedGraphControl1.GraphPane.AddCurve("Noise Remote", plotdatanoicer, Color.Orange, SymbolType.None);

            zedGraphControl1.GraphPane.Title.Text = "RSSI";

            if (Terminal.sw == null)
                Terminal.sw = new StreamWriter("Terminal-" + DateTime.Now.ToString("yyyy-MM-dd HH-mm-ss") + ".txt");
        }

        private void BUT_connect_Click(object sender, EventArgs e)
        {
            CustomMessageBox.Show("Ensure you disconnect properly, to leave the radio in a good state");

            try
            {
                MainV2.comPort.BaseStream.Open();

                inter.doConnect(MainV2.comPort.BaseStream);

                inter.doCommand(MainV2.comPort.BaseStream, "AT&T=RSSI");

                inter.doCommand(MainV2.comPort.BaseStream, "ATO");

                tickStart = Environment.TickCount;

                timer1.Start();

                BUT_disconnect.Enabled = true;
                BUT_connect.Enabled = false;

            }
            catch { CustomMessageBox.Show("Bad Port Setting"); }
        }

        private void BUT_disconnect_Click(object sender, EventArgs e)
        {
            try
            {
                timer1.Stop();

                inter.doConnect(MainV2.comPort.BaseStream);

                inter.doCommand(MainV2.comPort.BaseStream, "AT&T");

                inter.doCommand(MainV2.comPort.BaseStream, "ATO");

                MainV2.comPort.BaseStream.Close();

                BUT_disconnect.Enabled = false;
                BUT_connect.Enabled = true;
            }
            catch { }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (MainV2.comPort.BaseStream != null && MainV2.comPort.BaseStream.IsOpen)
            {
                MainV2.comPort.BaseStream.WriteLine("ABCDEFGHIJKLMNOPQRSTUVWXYZ");

                if (MainV2.comPort.BaseStream.BytesToRead < 50)
                    return;

                string line = MainV2.comPort.BaseStream.ReadLine();

                /*
L/R RSSI: 12/0  L/R noise: 17/0 pkts: 0  txe=0 rxe=0 stx=0 srx=0 ecc=0/0 temp=61 dco=0
L/R RSSI: 12/0  L/R noise: 16/0 pkts: 0  txe=0 rxe=0 stx=0 srx=0 ecc=0/0 temp=61 dco=0
                 */

                Regex rssi = new Regex(@"RSSI: ([0-9]+)/([0-9]+)\s+L/R noise: ([0-9]+)/([0-9]+)");

                Match match = rssi.Match(line);

                if (match.Success) {
                    double time = (Environment.TickCount - tickStart) / 1000.0;

                    plotdatarssil.Add(time, double.Parse(match.Groups[1].Value));
                    plotdatarssir.Add(time, double.Parse(match.Groups[2].Value));
                    plotdatanoicel.Add(time, double.Parse(match.Groups[3].Value));
                    plotdatanoicer.Add(time, double.Parse(match.Groups[4].Value));


                    // Make sure the Y axis is rescaled to accommodate actual data
                    zedGraphControl1.AxisChange();

                    // Force a redraw

                    zedGraphControl1.Invalidate();

                    if (Terminal.sw != null)
                    {
                        Terminal.sw.Write(line);
                        Terminal.sw.Flush();
                    }
                }

            }
        }
    }
}
