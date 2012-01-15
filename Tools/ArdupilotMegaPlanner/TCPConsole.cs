using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;

namespace ArdupilotMega
{
    static class TCPConsole
    {
        static TcpListener listener;
        static TcpClient client;
        static bool started = false;

        static void startup()
        {
            started = true;
            try
            {
                listener = new TcpListener(IPAddress.Any, 2300);

                listener.Start();

                listener.BeginAcceptTcpClient(
      new AsyncCallback(DoAcceptTcpClientCallback),
      listener);
            }
            catch { Console.WriteLine("TCP Console fail: port 2300"); return; }

        }

        // Process the client connection.
        static void DoAcceptTcpClientCallback(IAsyncResult ar)
        {
            // Get the listener that handles the client request.
            TcpListener listener = (TcpListener)ar.AsyncState;

            // End the operation and display the received data on 
            // the console.
            client = listener.EndAcceptTcpClient(ar);

            // setup for next listener
            listener.BeginAcceptTcpClient(
new AsyncCallback(DoAcceptTcpClientCallback),
listener);

            // Process the connection here. (Add the client to a
            // server table, read data, etc.)
            Console.WriteLine("Client connected completed");

            client.Client.NoDelay = true;
        }

        public static void Write(byte thing)
        {
            if (!started)
                startup();

            if (client == null)
                return;
            if (!client.Connected)
                return;
            try
            {
                client.GetStream().WriteByte(thing);
            }
            catch { }
        }
    }
}