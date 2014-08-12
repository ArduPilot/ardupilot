using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.Net;
using System.Net.Sockets;
using System.Text.RegularExpressions;

namespace SerialProxy
{
    public partial class Form1 : Form
    {
        static SerialPort comPort = new SerialPort();
        static int runthreads = 0;
        static TcpListener listener;
        static List<NetworkStream> clients = new List<NetworkStream>();

        public Form1()
        {
            InitializeComponent();

            Control.CheckForIllegalCrossThreadCalls = false; // so can update display from another thread

            Comports.Items.AddRange(SerialPort.GetPortNames());
            if (Comports.Items.Count > 0)
            {
                Comports.SelectedIndex = 0;
            }
        }

        private void ConnectComPort_Click(object sender, EventArgs e)
        {
            if (!comPort.IsOpen)
            {
                outputlog.Clear();
                outputlog.ClearUndo();

                comPort.BaudRate = int.Parse(baudrate.Text.ToString());
                comPort.DataBits = 8;
                comPort.StopBits = (StopBits)Enum.Parse(typeof(StopBits), "1");
                comPort.Parity = (Parity)Enum.Parse(typeof(Parity), "None");

                try
                {
                    comPort.PortName = Comports.Text.ToString();
                    comPort.DtrEnable = false; // dont reset yet
                    comPort.Open();
                    StatusCom.Text = "Com Connected";
                    ConnectComPort.Text = "Disconnect";
                }
                catch (Exception) { outputlog.AppendText("Cant open serial port\r\n"); return; }

                try
                {

                    listener = new TcpListener(IPAddress.Any, int.Parse(tcpport.Text.ToString()));


                    StatusTCP.Text = "TCP Waiting";

                    runthreads = 1;

                    System.Threading.Thread t11 = new System.Threading.Thread(delegate() { try { mainloop(); } catch (Exception) { } }); // process serial data and send to clients
                    t11.Start();

                    System.Threading.Thread t12 = new System.Threading.Thread(delegate() { try { listernforclients(); } catch (Exception) { } }); // wait for tcp connections
                    t12.Start();


                }
                catch (SocketException ex)
                {
                    outputlog.AppendText("SocketException: " + ex+ "\n");
                }
                finally
                {
                    // Stop listening for new clients.
                    listener.Stop();
                }

            }
            else
            {
                runthreads = 0;
                System.Threading.Thread.Sleep(100); // make sure thread closes
                comPort.Close();
                StatusCom.Text = "Com Disconnected";
                ConnectComPort.Text = "Connect";
                foreach (NetworkStream client in clients)
                {
                    try
                    {
                        client.Close();
                    }
                    catch (Exception)
                    {
                    }
                }
                clients.Clear();
                StatusTCP.Text = "TCP Disconnected";
            }
        }

        void listernforclients()
        {
            listener.Start();
            // Enter the listening loop.

            while (runthreads == 1)
            {

                // Perform a blocking call to accept requests.
                // You could also user server.AcceptSocket() here.
                TcpClient client = listener.AcceptTcpClient();
                StatusTCP.Text = "TCP " + (clients.Count +1) + " Clients";

                comPort.DtrEnable = CHK_reset.Checked;

                // Get a stream object for reading and writing
                NetworkStream stream = client.GetStream();

                clients.Add(stream);

                System.Threading.Thread.Sleep(100);

            }
        }

        void mainloop()
        {
            System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
            List<NetworkStream> clientscopy = new List<NetworkStream>(clients);
            byte[] data = new byte[1024 * 4];

            while (runthreads == 1)
            {
                System.Threading.Thread.Sleep(1);
                // do serial
                if (comPort.BytesToRead > 0)
                {
                    string line = comPort.ReadExisting();

                    if (convertCRLF.Checked)
                    {
                        Regex regex = new Regex("\r\n", RegexOptions.IgnoreCase);

                        line = regex.Replace(line, "\0");
                    }

                    outputlog.AppendText(line);

                    clientscopy = new List<NetworkStream>(clients);

                    foreach (NetworkStream client in clientscopy)
                    {
                        byte[] temp = encoding.GetBytes(line);
                        try
                        {
                            client.Write(temp, 0, temp.Length);
                        }
                        catch (Exception)
                        {
                            clients.Remove(client);
                            StatusTCP.Text = "TCP " + clients.Count + " Clients";
                        }
                    }
                }
                // do tcp
                clientscopy = new List<NetworkStream>(clients);

                foreach (NetworkStream client in clientscopy)
                {
                    //byte[] temp = encoding.GetBytes(data);
                    if (client.DataAvailable)
                    {
                        try
                        {
                            int size = client.Read(data, 0, data.Length);
                            comPort.Write(data, 0, size);
                        }
                        catch (Exception)
                        {
                            clients.Remove(client);
                            StatusTCP.Text = "TCP " + clients.Count + " Clients";
                        }
                    }
                }

            }
        }

        private void Comports_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void Comports_Click(object sender, EventArgs e)
        {
            Comports.Items.Clear();
            Comports.Items.AddRange(SerialPort.GetPortNames());
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            runthreads = 0;
            System.Threading.Thread.Sleep(100);
            if (comPort.IsOpen)
                comPort.Close();
            try
            {
                listener.Stop();
            }
            catch (Exception) { }
        }

        private void CHK_reset_Click(object sender, EventArgs e)
        {
            if (CHK_reset.Checked)
            {
                CHK_reset.Checked = false;
            }
            else
            {
                CHK_reset.Checked = true;
            }
        }

        private void toolStripMenuItem3_Click(object sender, EventArgs e)
        {
            if (convertCRLF.Checked)
            {
                convertCRLF.Checked = false;
            }
            else
            {
                convertCRLF.Checked = true;
            }
        }

        private void outputlog_TextChanged(object sender, EventArgs e)
        {

            if (outputlog.Text.Length > 10000)
            {
                outputlog.Text = outputlog.Text.Remove(0, 5000); // half size
            }

            // auto scroll
            outputlog.SelectionStart = outputlog.Text.Length;

            outputlog.ScrollToCaret();

            outputlog.Refresh();
        }
    }
}
