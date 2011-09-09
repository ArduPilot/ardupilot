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
using System.Threading;
using System.Net.NetworkInformation;

// SerialPort > CommsServer > NetSerial > Mavlink > Application

namespace SerialProxy
{
    public class CommsServer
    {
        SerialPort comPort = new SerialPort();
        public int runthreads = 0;
        Socket listener = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        public List<Socket> clients = new List<Socket>();
        public int tcpport = 59001;
        bool doDTR = false;
        Thread t11;
        Thread t12;
        bool firstconnect = false;

        public void toggleDTR(bool doit)
        {
            doDTR = doit;
        }

        public void toggleDTRnow()
        {
            comPort.DtrEnable = !doDTR;
            System.Threading.Thread.Sleep(100);
            comPort.DtrEnable = doDTR;
        }

        // from http://stackoverflow.com/questions/570098/in-c-how-to-check-if-a-tcp-port-is-available
        public static bool CheckAvailableServerPort(int port) 
        {     
            bool isAvailable = true;      
            // Evaluate current system tcp connections. This is the same information provided     
            // by the netstat command line application, just in .Net strongly-typed object     
            // form.  We will look through the list, and if our port we would like to use     
            // in our TcpClient is occupied, we will set isAvailable to false.    
            try
            {
                IPGlobalProperties ipGlobalProperties = IPGlobalProperties.GetIPGlobalProperties();
                IPEndPoint[] tcpConnInfoArray = ipGlobalProperties.GetActiveTcpListeners();
                foreach (IPEndPoint endpoint in tcpConnInfoArray)
                {
                    if (endpoint.Port == port)
                    {
                        isAvailable = false;
                        break;
                    }
                }
            }
            catch { return true; } // this fails on unix... so we just accept
            return isAvailable;
        } 

        ~CommsServer()
        {
            this.shutdown();
            Console.WriteLine("CommsServer Destroyed!!!");
        }

        public CommsServer(int portno)
        {
            tcpport = portno;
        }
        
        public void Start(string comport,int baudrate)
        {
            Console.WriteLine("CommsServer Init");

            if (!comPort.IsOpen)
            {
                Console.WriteLine("CommsServer set com setting");
                comPort.BaudRate = baudrate;
                comPort.DataBits = 8;
                comPort.StopBits = StopBits.One;
                comPort.Parity = Parity.None;

                try
                {
                    comPort.PortName = comport;

                    runthreads = 1;
                    Console.WriteLine("CommsServer threads");
                    t11 = new Thread(delegate() { try { mainloop(); } catch (Exception e) { Console.WriteLine("Serial Error " + e.ToString()); } }); // process serial data and send to clients
                    t11.Name = "CommsServer Serial";
                    t11.IsBackground = true;
                    t11.Start();
                    ArdupilotMega.MainV2.threads.Add(t11);

                    t12 = new Thread(delegate() { try { listernforclients(); } catch (Exception e) { Console.WriteLine("Socket Error " + e.ToString()); } }); // wait for tcp connections
                    t12.Name = "CommsServer Socket";
                    t12.IsBackground = true;
                    t12.Start();
                    ArdupilotMega.MainV2.threads.Add(t12);

                    Console.WriteLine("CommsServer set netserial.port");
                    NetSerial.port = this;

                    int timeout = 0;

                    while (comPort.IsOpen == false || listener.IsBound == false)
                    {
                        if (timeout > 200) { // timeout after 4 sec 200 * 20
                            this.shutdown();
                            throw new Exception("Timeout connecting port, Bad Settings or port in use");
                        }

                        System.Threading.Thread.Sleep(20); // allow threads to start

                        timeout++;
                    }
                }
                catch (Exception ex)
                {
                    this.shutdown();
                    throw new Exception("CommServer Fail: " + ex.Message + "\n");
                }
            }
            Console.WriteLine("CommsServer Started");
        }

        public void shutdown()
        {
            Console.WriteLine("CommsServer Shutdown");
            runthreads = 0;

            try
            {
                listener.Close();
            }
            catch { }
            try
            {
                if (comPort != null && comPort.IsOpen)
                    comPort.Close();
                comPort = null;
            }
            catch { }
            try
            {
                List<Socket> clientscopy = new List<Socket>(clients);

                foreach (Socket client in clientscopy)
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
            }
            catch { shutdown(); }

            System.Threading.Thread.Sleep(500);

            NetSerial.port = null;
        }

        void listernforclients()
        {
            Console.WriteLine("CommsServer listener");

            IPEndPoint ipep = new IPEndPoint(IPAddress.Loopback, tcpport);

            listener = new Socket(AddressFamily.InterNetwork,
                            SocketType.Stream, ProtocolType.Tcp);

            listener.Bind(ipep);

            listener.Listen(10);

            // Enter the listening loop.

            while (runthreads == 1)
            {
                try
                {
                    Console.WriteLine("CommsServer listern wait");
                    // Perform a blocking call to accept requests.
                    Socket client = listener.Accept();

                    Console.WriteLine("CommsServer listern accept");

                    comPort.DtrEnable = doDTR;

                    clients.Add(client);

                    System.Threading.Thread.Sleep(100);

                    firstconnect = true;
                }
                catch (Exception e) { Console.WriteLine("CommServer listener error : "+ e.Message); } // cant exit
            }
            listener.Close();

            shutdown();
        }

        void mainloop()
        {
            System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
            List<Socket> clientscopy = new List<Socket>(clients);
            byte[] data = new byte[1024 * 4];

            try
            {
                comPort.ReadBufferSize = 100 * 1024;
                comPort.Open();
            }
            catch (Exception e) { MessageBox.Show("CommsServer Error opening Com Port " + e.Message); this.shutdown(); return; }

            Console.WriteLine("CommsServer comPort Opened");

            while (runthreads == 1)
            {
                try
                {
                    if (NetSerial.port == null) // make sure we always keep track of this
                    {
                        NetSerial.port = this;
                    }

                    if (comPort == null || !comPort.IsOpen)
                    {
                        Console.WriteLine("CommServer error : Closing");
                        runthreads = 0;
                        this.shutdown();
                        throw new Exception("CommServer : Comport Closed or null");
                        //return;
                    }
                    // do serial
                    while (comPort.BytesToRead > 0)
                    {
                        //Console.Write("BTR " + comPort.BytesToRead + "\r");
                        byte[] buffer = new byte[comPort.BytesToRead];

                        comPort.Read(buffer, 0, buffer.Length);

                        clientscopy = new List<Socket>(clients);

                        foreach (Socket client in clientscopy)
                        {
                            try
                            {
                                client.Send(buffer, 0, buffer.Length,SocketFlags.None);
                            }
                            catch
                            {
                                Console.WriteLine("CommsServer closing client ");
                                if (client != null)
                                    client.Close();
                                clients.Remove(client);
                            }
                        }
                        System.Threading.Thread.Sleep(2); // this gives tme to hopefully be outside the main apm loop
                    }
                    // do tcp
                    if (clients.Count == 0 && firstconnect)
                    {
                        shutdown();
                        return;
                    }


                    // 57600 / 8 = 1000 / 7200 = 0.1388888888888889 ms per char
                    // 20ms per cycle = 144 bytes per cycle - avg 2500 bps
//                    System.Threading.Thread.Sleep(2);

                    clientscopy = new List<Socket>(clients);

                    foreach (Socket client in clientscopy)
                    {
                        //Console.WriteLine("NS BTR " + client.DataAvailable);
                        //byte[] temp = encoding.GetBytes(data);
                        if (client.Available != 0)
                        {
                            try
                            {
                                int size = client.Receive(data, 0, data.Length,SocketFlags.None);
                                //Console.WriteLine("TCP to Serial {0}", size);
                                comPort.Write(data, 0, size);
                            }
                            catch
                            {
                                Console.WriteLine("CommsServer closing client ");
                                if (client != null)
                                    client.Close();
                                clients.Remove(client);
                            }
                        } // if
                        if (SocketConnected(client) == false)
                        {
                            Console.WriteLine("CommsServer closing client - Remote close");
                            if (client != null)
                                client.Close();
                            clients.Remove(client);
                        }
                    } // foreach
                    System.Threading.Thread.Sleep(1);
                } // try
                catch (Exception e) { Console.WriteLine("CommServer serial error : " + e.ToString()); } // cant exit

            } // while
        }

        bool SocketConnected(Socket s)
        {
            bool part1 = s.Poll(1000, SelectMode.SelectRead);
            bool part2 = (s.Available == 0);
            if (part1 & part2)
            {//connection is closed 
                return false;
            }
            return true;
        } 

    }
}
