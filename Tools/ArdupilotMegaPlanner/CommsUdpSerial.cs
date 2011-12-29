using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.IO.Ports;
using System.Threading;
using System.Net; // dns, ip address
using System.Net.Sockets; // tcplistner

namespace System.IO.Ports
{
    public class UdpSerial : ArdupilotMega.ICommsSerial
    {
        UdpClient client = new UdpClient();
        IPEndPoint RemoteIpEndPoint = new IPEndPoint(IPAddress.Any, 0);
        byte[] rbuffer = new byte[0];
        int rbufferread = 0;

        public int WriteBufferSize { get; set; }
        public int WriteTimeout { get; set; }
        public int ReceivedBytesThreshold { get; set; }
        public bool RtsEnable { get; set; }

        ~UdpSerial()
        {
            this.Close();
            client = null;
        }

        public UdpSerial()
        {
            //System.Threading.Thread.CurrentThread.CurrentUICulture = new System.Globalization.CultureInfo("en-US");
            //System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-US");

            Port = "14550";
        }

        public void toggleDTR()
        {
        }

        public string Port { get; set; }

        public int ReadTimeout
        {
            get;// { return client.ReceiveTimeout; }
            set;// { client.ReceiveTimeout = value; }
        }

        public int ReadBufferSize {get;set;}

        public int BaudRate { get; set; }
        public StopBits StopBits { get; set; }
        public  Parity Parity { get; set; }
        public  int DataBits { get; set; }

        public string PortName { get; set; }

        public  int BytesToRead
        {
            get { return client.Available + rbuffer.Length - rbufferread; }
        }

        public bool IsOpen { get { return client.Client.Connected; } }

        public bool DtrEnable
        {
            get;
            set;
        }

        public  void Open()
        {
            if (client.Client.Connected)
            {
                Console.WriteLine("udpserial socket already open");
                return;
            }

            string dest = Port;
            ArdupilotMega.Common.InputBox("Listern Port", "Enter Local port (ensure remote end is already sending)", ref dest);
            Port = dest;

            client = new UdpClient(int.Parse(Port));

                         int timeout = 5;  
            while (timeout > 0) 
            { 
                if (BytesToRead > 0)  
                    break;  
                System.Threading.Thread.Sleep(1000);  
                timeout--;  
            }  
            if (BytesToRead == 0)  
                return; 

            try
            {
                client.Receive(ref RemoteIpEndPoint);
                Console.WriteLine("NetSerial connecting to {0} : {1}", RemoteIpEndPoint.Address, RemoteIpEndPoint.Port);
                client.Connect(RemoteIpEndPoint);
            }
            catch (Exception e) { 
                if (client != null && client.Client.Connected) { client.Close(); }
                Console.WriteLine(e.ToString());
                System.Windows.Forms.MessageBox.Show("Please check your Firewall settings\nPlease try running this command\n1.    Run the following command in an elevated command prompt to disable Windows Firewall temporarily:\n    \nNetsh advfirewall set allprofiles state off\n    \nNote: This is just for test; please turn it back on with the command 'Netsh advfirewall set allprofiles state on'.\n");
                throw new Exception("The socket/serialproxy is closed " + e);
            }

            return;
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

                if (rbufferread == rbuffer.Length)
                {
                    MemoryStream r = new MemoryStream();
                    while (client.Available > 0)
                    {
                        Byte[] b = client.Receive(ref RemoteIpEndPoint);
                        r.Write(b, 0, b.Length);
                    }
                    rbuffer = r.ToArray();
                    rbufferread = 0;
                }

                Array.Copy(rbuffer, rbufferread, readto, offset, length);

                rbufferread += length;

                return length;
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
            byte[] buffer = new byte[1];
            Read(buffer, 0, 1);
            return buffer[0];
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
                client.Send(write, length);
            }
            catch { }//throw new Exception("Comport / Socket Closed"); }
        }

        public  void DiscardInBuffer()
        {
            VerifyConnected();
            int size = client.Available;
            byte[] crap = new byte[size];
            Console.WriteLine("UdpSerial DiscardInBuffer {0}",size);
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
            if (client.Client.Connected)
            {
                client.Client.Close();
                client.Close();
            }

            client = new UdpClient();
        }
    }
}
