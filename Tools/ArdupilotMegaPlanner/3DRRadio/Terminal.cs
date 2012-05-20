using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using ArdupilotMega;
using ArdupilotMega.Comms;
using System.IO;

namespace _3DRRadio
{
    public partial class Terminal : UserControl
    {
        ICommsSerial comPort = MainV2.comPort.BaseStream;
        Object thisLock = new Object();
        public static bool threadrun = false;
        StringBuilder cmd = new StringBuilder();
        internal static StreamWriter sw;

        public Terminal()
        {
            InitializeComponent();

            if (sw == null)
                sw = new StreamWriter("Terminal-" + DateTime.Now.ToString("yyyy-MM-dd HH-mm-ss") + ".txt");
        }

        void comPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (!comPort.IsOpen)
                return;
            try
            {
                lock (thisLock)
                {
                    string data = comPort.ReadExisting();
                    //Console.Write(data);

                    if (sw != null)
                    {
                        sw.Write(data);
                        sw.Flush();
                    }

                    addText(data);
                }
            }
            catch (Exception) { if (!threadrun) return; TXT_terminal.AppendText("Error reading com port\r\n"); }
        }

        void addText(string data)
        {
            this.Invoke((System.Windows.Forms.MethodInvoker)delegate()
            {
                TXT_terminal.SelectionStart = TXT_terminal.Text.Length;

                data = data.TrimEnd('\r'); // else added \n all by itself
                data = data.Replace("\0", " ");
                TXT_terminal.AppendText(data);
                if (data.Contains("\b"))
                {
                    TXT_terminal.Text = TXT_terminal.Text.Remove(TXT_terminal.Text.IndexOf('\b'));
                    TXT_terminal.SelectionStart = TXT_terminal.Text.Length;
                }
            });
        }

        private void Terminal_Load(object sender, EventArgs e)
        {
            try
            {

                if (comPort.IsOpen)
                    comPort.Close();

                comPort.ReadBufferSize = 1024 * 1024;

                comPort.PortName = MainV2.comPort.BaseStream.PortName;

                comPort.Open();

                comPort.toggleDTR();

                System.Threading.Thread t11 = new System.Threading.Thread(delegate()
                {
                    threadrun = true;

                    DateTime start = DateTime.Now;

                    while ((DateTime.Now - start).TotalMilliseconds < 2000)
                    {
                        try
                        {
                            if (comPort.BytesToRead > 0)
                            {
                                comPort_DataReceived((object)null, (SerialDataReceivedEventArgs)null);
                            }
                        }
                        catch { return; }
                    }
                    try
                    {
                        comPort.Write("\n\n\n");
                    }
                    catch { return; }
                    while (threadrun)
                    {
                        try
                        {
                            System.Threading.Thread.Sleep(10);
                            if (!comPort.IsOpen)
                                break;
                            if (comPort.BytesToRead > 0)
                            {
                                comPort_DataReceived((object)null, (SerialDataReceivedEventArgs)null);
                            }
                        }
                        catch { }
                    }

                    comPort.DtrEnable = false;

                    try
                    {
                        //if (sw != null)
                          //  sw.Close();
                    }
                    catch { }

                    if (threadrun == false)
                    {
                        comPort.Close();
                    }
                    Console.WriteLine("Comport thread close");
                });
                t11.IsBackground = true;
                t11.Name = "Terminal serial thread";
                t11.Start();

                // doesnt seem to work on mac
                //comPort.DataReceived += new SerialDataReceivedEventHandler(comPort_DataReceived);

                TXT_terminal.AppendText("Opened com port\r\n");
            }
            catch (Exception) { TXT_terminal.AppendText("Cant open serial port\r\n"); return; }

            TXT_terminal.Focus();
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
            if (e.KeyData == Keys.Up || e.KeyData == Keys.Down || e.KeyData == Keys.Left || e.KeyData == Keys.Right)
            {
                e.Handled = true; // ignore it
            }
        }

        private void Terminal_FormClosing(object sender, FormClosingEventArgs e)
        {
            threadrun = false;

            if (comPort.IsOpen)
            {
                comPort.Close();
            }
            System.Threading.Thread.Sleep(400);
        }

        private void TXT_terminal_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == '\r')
            {
                if (comPort.IsOpen)
                {
                    try
                    {
                        // do not change this  \r is correct - no \n
                        string temp = cmd.ToString();

                        if (cmd.ToString() == "+++")
                        {
                            comPort.Write(Encoding.ASCII.GetBytes(cmd.ToString()), 0, cmd.Length);
                        }
                        else
                        {
                            comPort.Write(Encoding.ASCII.GetBytes(cmd.ToString() + "\r"), 0, cmd.Length + 1);
                        }

                        if (sw != null)
                        {
                            sw.WriteLine(cmd.ToString());
                            sw.Flush();
                        }
                    }
                    catch { CustomMessageBox.Show("Error writing to com port"); }
                }
                cmd = new StringBuilder();
            }
            else
            {
                cmd.Append(e.KeyChar);
            }
        }
    }
}
