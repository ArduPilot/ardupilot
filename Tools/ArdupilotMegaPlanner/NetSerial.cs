using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.IO.Ports;
using System.Threading;
using System.Net; // dns, ip address
using System.Net.Sockets; // tcplistner
using SerialProxy;

namespace System.IO.Ports
{
    public class NetSerial // : SerialPort
    {
        TcpClient client = new TcpClient();
        public static CommsServer port;

        ~NetSerial()
        {
            this.Close();
            client = null;
        }

        public NetSerial()
        {
            //System.Threading.Thread.CurrentThread.CurrentUICulture = new System.Globalization.CultureInfo("en-US");
            //System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-US");

            PortDest = "127.0.0.1:59001";
        }

        public string PortDest { get; set; }

        public int ReadTimeout
        {
            get;// { return client.ReceiveTimeout; }
            set;// { client.ReceiveTimeout = value; }
        }

        public int ReadBufferSize { 
            get { return client.ReceiveBufferSize; }
            set { client.ReceiveBufferSize = value; }
        }

        public int BaudRate { get; set; }
        public StopBits StopBits { get; set; }
        public  Parity Parity { get; set; }
        public  int DataBits { get; set; }

        public string PortName { get; set; }

        public  int BytesToRead
        {
            get { return client.Available; }
        }

        public bool IsOpen { get { return client.Connected; } }

        public bool DtrEnable
        {
            get;
            set;
        }

        public  bool Open()
        {
            if (client.Connected)
            {
                Console.WriteLine("netserial socket already open");
                return true;
            }

            string host = PortDest.Substring(0, PortDest.IndexOf(":"));
            int port = int.Parse(PortDest.Substring(PortDest.IndexOf(":") + 1));

            if (NetSerial.port == null)
            {
                while (!CommsServer.CheckAvailableServerPort(port))
                {
                    Console.WriteLine("CommsServer port not avaliable {0}", port);
                    port++;
                }
                NetSerial.port = new SerialProxy.CommsServer(port);
                NetSerial.port.Start(this.PortName,this.BaudRate);
            }

            // set port - including if commserver started
            port = NetSerial.port.tcpport;

            NetSerial.port.toggleDTR(DtrEnable);

            try
            {
                Console.WriteLine("NetSerial connecting to {0} : {1}",host,port);
                client.Connect(host, port);
                client.NoDelay = true;
            }
            catch (Exception e) { 
                if (client != null && client.Connected) { client.Close(); }
                Console.WriteLine(e.ToString());
                System.Windows.Forms.MessageBox.Show("Please check your Firewall settings\nPlease try running this command\n1.    Run the following command in an elevated command prompt to disable Windows Firewall temporarily:\n    \nNetsh advfirewall set allprofiles state off\n    \nNote: This is just for test; please turn it back on with the command 'Netsh advfirewall set allprofiles state on'.\n");
                throw new Exception("The socket/serialproxy is closed " + e);
            }

            return IsOpen;
        }

        void VerifyConnected()
        {
            if (client == null || !IsOpen)
            {
                throw new Exception("The socket/serialproxy is closed");
            }
        }

        public  int Read(byte[] readto,int offset,int length)
        {
            VerifyConnected();
            try
            {
                if (length < 1) { return 0; }

                return client.GetStream().Read(readto, offset, length);
            }
            catch { throw new Exception("Socket Closed"); }
        }

        public  int ReadByte()
        {
            VerifyConnected();
            int count = 0;
            while (this.BytesToRead == 0)
            {
                System.Threading.Thread.Sleep(1);
                if (count > ReadTimeout)
                    throw new Exception("NetSerial Timeout on read");
                count++;
            }
            return client.GetStream().ReadByte();
        }

        public  int ReadChar()
        {
            return ReadByte();
        }

        public  string ReadExisting() 
        {
            VerifyConnected();
            byte[] data = new byte[client.Available];
            if (data.Length > 0)
                Read(data, 0, data.Length);

            string line = Encoding.ASCII.GetString(data, 0, data.Length);

            return line;
        }

        public  void WriteLine(string line)
        {
            VerifyConnected();
            line = line + "\n";
            Write(line);
        }

        public  void Write(string line)
        {
            VerifyConnected();
            byte[] data = new System.Text.ASCIIEncoding().GetBytes(line);
            Write(data, 0, data.Length);
        }

        public  void Write(byte[] write, int offset, int length)
        {
            VerifyConnected();
            try
            {
                client.GetStream().Write(write, offset, length);
            }
            catch { }//throw new Exception("Comport / Socket Closed"); }
        }

        public  void DiscardInBuffer()
        {
            VerifyConnected();
            int size = client.Available;
            byte[] crap = new byte[size];
            Console.WriteLine("NetSerial DiscardInBuffer {0}",size);
            Read(crap, 0, size);
        }

        public  string ReadLine() {
            byte[] temp = new byte[4000];
            int count = 0;
            int timeout = 0;

            while (timeout <= 100)
            {
                if (!this.IsOpen) { break; }
                if (this.BytesToRead > 0)
                {
                    byte letter = (byte)this.ReadByte();

                    temp[count] = letter;

                    if (letter == '\n') // normal line
                    {
                        break;
                    }


                    count++;
                    if (count == temp.Length)
                        break;
                    timeout = 0;
                } else {
                    timeout++;
                    System.Threading.Thread.Sleep(5);
                }
            }

            Array.Resize<byte>(ref temp, count + 1);

            return Encoding.ASCII.GetString(temp, 0, temp.Length);
        }

        public void Close()
        {
            if (client.Connected)
            {
                client.Client.Close();
                client.Close();
            }

            client = new TcpClient();

            DtrEnable = false;

            System.Threading.Thread.Sleep(100);
        }
    }
}
