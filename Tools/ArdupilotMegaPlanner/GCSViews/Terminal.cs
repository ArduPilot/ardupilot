using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ArdupilotMega;
using System.IO.Ports;
using ArdupilotMega.Comms;
using ArdupilotMega.Utilities;
using System.Text.RegularExpressions;

namespace ArdupilotMega.GCSViews
{
    public partial class Terminal : MyUserControl
    {
        ICommsSerial comPort = MainV2.comPort.BaseStream;
        Object thisLock = new Object();
        public static bool threadrun = false;
        bool inlogview = false;
        List<string> cmdHistory = new List<string>();
        int history = 0;
        int inputStartPos = 0;

        public Terminal()
        {
            threadrun = false;

            InitializeComponent();
        }

        void comPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (!comPort.IsOpen)
                return;

            // if btr > 0 then this shouldnt happen
            comPort.ReadTimeout = 300;

            try
            {
                lock (thisLock)
                {
                    System.Threading.Thread.Sleep(20);
                    byte[] buffer = new byte[256];
                    int a = 0;

                    while (comPort.BytesToRead > 0)
                    {
                        byte indata = (byte)comPort.ReadByte();

                        buffer[a] = indata;

                        if (buffer[a] >= 0x20 && buffer[a] < 0x7f || buffer[a] == (int)'\n' || buffer[a] == (int)'\r')
                        {
                            a++;
                        }
                        if (a == (buffer.Length-1))
                            break;
                    }

                    addText(ASCIIEncoding.ASCII.GetString(buffer,0,a+1));
                }
            }
            catch (Exception ex) { Console.WriteLine(ex.ToString()); if (!threadrun) return; TXT_terminal.AppendText("Error reading com port\r\n"); }
        }

        void addText(string data)
        {
            this.BeginInvoke((System.Windows.Forms.MethodInvoker)delegate()
            {
                TXT_terminal.SelectionStart = TXT_terminal.Text.Length;

                data = data.Replace("U3","");
                data = data.Replace("U$", "");
                data = data.Replace(@"U""","");
                data = data.Replace("d'`F", "");
                data = data.Replace("U.", "");
                data = data.Replace("'`","");

                data = data.TrimEnd('\r'); // else added \n all by itself
                data = data.Replace("\0", "");
                TXT_terminal.AppendText(data);
                if (data.Contains("\b"))
                {
                    TXT_terminal.Text = TXT_terminal.Text.Remove(TXT_terminal.Text.IndexOf('\b'));
                    TXT_terminal.SelectionStart = TXT_terminal.Text.Length;
                }
                inputStartPos = TXT_terminal.SelectionStart;
            });

        }

        private void TXT_terminal_Click(object sender, EventArgs e)
        {
            // auto scroll
            TXT_terminal.SelectionStart = TXT_terminal.Text.Length;

            TXT_terminal.ScrollToCaret();

            TXT_terminal.Refresh();
        }

        private void TXT_terminal_KeyDown(object sender, KeyEventArgs e)
        {
            /*    if (e.KeyData == Keys.Up || e.KeyData == Keys.Down || e.KeyData == Keys.Left || e.KeyData == Keys.Right)
                {
                    e.Handled = true; // ignore it
                }*/
            lock (thisLock)
            {
                switch (e.KeyData)
                {
                    case Keys.Up:
                        if (history > 0)
                        {
                            TXT_terminal.Select(inputStartPos, TXT_terminal.Text.Length - inputStartPos);
                            TXT_terminal.SelectedText = "";
                            TXT_terminal.AppendText(cmdHistory[--history]);
                        }
                        e.Handled = true;
                        break;
                    case Keys.Down:
                        if (history < cmdHistory.Count - 1)
                        {
                            TXT_terminal.Select(inputStartPos, TXT_terminal.Text.Length - inputStartPos);
                            TXT_terminal.SelectedText = "";
                            TXT_terminal.AppendText(cmdHistory[++history]);
                        }
                        e.Handled = true;
                        break;
                    case Keys.Left:
                    case Keys.Back:
                        if (TXT_terminal.SelectionStart <= inputStartPos)
                            e.Handled = true;
                        break;

                    //case Keys.Right:
                    //    break;
                }
            }
        }

        private void Terminal_FormClosing(object sender, FormClosingEventArgs e)
        {
            threadrun = false;

            try
            {
                if (comPort.IsOpen)
                {
                    comPort.Close();
                }
            }
            catch { } // Exception System.IO.IOException: The specified port does not exist.

            System.Threading.Thread.Sleep(400);

            MainV2.giveComport = false;
        }

        private void TXT_terminal_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == '\r')
            {
                if (comPort.IsOpen)
                {
                    try
                    {
                        string cmd = "";
                        lock (thisLock)
                        {
                            if (MainV2.MONO)
                            {
                                cmd = TXT_terminal.Text.Substring(inputStartPos, TXT_terminal.Text.Length - inputStartPos);
                            }
                            else
                            {
                                cmd = TXT_terminal.Text.Substring(inputStartPos, TXT_terminal.Text.Length - inputStartPos - 1);
                            }
                            TXT_terminal.Select(inputStartPos, TXT_terminal.Text.Length - inputStartPos);
                            TXT_terminal.SelectedText = "";
                            if (cmd.Length > 0 && (cmdHistory.Count == 0 || cmdHistory.Last() != cmd))
                            {
                                cmdHistory.Add(cmd);
                                history = cmdHistory.Count;
                            }
                        }
                        // do not change this  \r is correct - no \n
                        if (cmd == "+++")
                        {
                            comPort.Write(Encoding.ASCII.GetBytes(cmd), 0, cmd.Length);
                        }
                        else
                        {
                            comPort.Write(Encoding.ASCII.GetBytes(cmd + "\r"), 0, cmd.Length + 1);
                        }
                    }
                    catch { CustomMessageBox.Show("Error writing to com port"); }
                }
            }
            /*
            if (comPort.IsOpen)
            {
                try
                {
                    comPort.Write(new byte[] { (byte)e.KeyChar }, 0, 1);
                }
                catch { MessageBox.Show("Error writing to com port"); }
            }
            e.Handled = true;*/
        }

        private void waitandsleep(int time)
        {
            DateTime start = DateTime.Now;

            while ((DateTime.Now - start).TotalMilliseconds < time)
            {
                try
                {
                    if (comPort.BytesToRead > 0)
                    {
                        return;
                    }
                }
                catch { threadrun = false; return; }
            }
        }

        private void readandsleep(int time)
        {
             DateTime start = DateTime.Now;

                    while ((DateTime.Now - start).TotalMilliseconds < time)
                    {
                        try
                        {
                            if (comPort.BytesToRead > 0)
                            {
                                comPort_DataReceived((object)null, (SerialDataReceivedEventArgs)null);
                            }
                        }
                        catch { threadrun = false;  return; }
                    }
        }

        private void Terminal_Load(object sender, EventArgs e)
        {
            try
            {
                MainV2.giveComport = true;

                if (comPort.IsOpen)
                    comPort.Close();

                comPort.ReadBufferSize = 1024 * 1024;

                comPort.PortName = MainV2.comPortName;

                comPort.Open();

                comPort.toggleDTR();

                comPort.DiscardInBuffer();

                System.Threading.Thread t11 = new System.Threading.Thread(delegate()
                {
                    threadrun = true;

                    // 10 sec
                    waitandsleep(10000);

                    // 100 ms
                    readandsleep(100);
                    try
                    {
                        comPort.Write("\n\n\n");

                        // 1 secs
                        readandsleep(1000);

                        comPort.Write("\r\r\r?\r");
                    }
                    catch { threadrun = false; return; }                  

                    while (threadrun)
                    {
                        try
                        {
                            System.Threading.Thread.Sleep(10);
                            if (inlogview)
                                continue;
                            if (!comPort.IsOpen)
                                break;
                            if (comPort.BytesToRead > 0)
                            {
                                comPort_DataReceived((object)null, (SerialDataReceivedEventArgs)null);
                            }
                        }
                        catch { }
                    }

                    threadrun = false;

                    comPort.DtrEnable = false;

                    comPort.Close();

                    Console.WriteLine("Comport thread close");
                });
                t11.IsBackground = true;
                t11.Name = "Terminal serial thread";
                t11.Start();

                // doesnt seem to work on mac
                //comPort.DataReceived += new SerialDataReceivedEventHandler(comPort_DataReceived);

                TXT_terminal.AppendText("Opened com port\r\n");
                inputStartPos = TXT_terminal.SelectionStart;
            }
            catch (Exception) { TXT_terminal.AppendText("Cant open serial port\r\n"); return; }

            TXT_terminal.Focus();
        }

        private void BUTsetupshow_Click(object sender, EventArgs e)
        {
            if (comPort.IsOpen)
            {
                try
                {
                    System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
                    byte[] data = encoding.GetBytes("exit\rsetup\rshow\r");
                    comPort.Write(data, 0, data.Length);
                }
                catch { }
            }
            TXT_terminal.Focus();
        }

        private void BUTradiosetup_Click(object sender, EventArgs e)
        {
            if (comPort.IsOpen)
            {
                try
                {
                    System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
                    byte[] data = encoding.GetBytes("exit\rsetup\r\nradio\r");
                    comPort.Write(data, 0, data.Length);
                }
                catch { }
            }
            TXT_terminal.Focus();
        }

        private void BUTtests_Click(object sender, EventArgs e)
        {
            if (comPort.IsOpen)
            {
                try
                {
                    System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
                    byte[] data = encoding.GetBytes("exit\rtest\r?\r\n");
                    comPort.Write(data, 0, data.Length);
                }
                catch { }
            }
            TXT_terminal.Focus();
        }

        private void Logs_Click(object sender, EventArgs e)
        {
            Form Log = new Log();
            ThemeManager.ApplyThemeTo(Log);
            inlogview = true;
            Log.ShowDialog();
            inlogview = false;
        }

        private void BUT_logbrowse_Click(object sender, EventArgs e)
        {
            Form logbrowse = new LogBrowse();
            ThemeManager.ApplyThemeTo(logbrowse);
            logbrowse.ShowDialog();
        }
    }
}