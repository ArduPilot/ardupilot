// Unix domain socket terminal for host-side ArduPilot tools. Renode's stock
// server terminal only accepts TCP port numbers.
using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;

namespace Antmicro.Renode.Backends.Terminals
{
    public static class AP_UnixSocketTerminalExtensions
    {
        public static void CreateUnixSocketTerminal(this Emulation emulation,
            string path, string name)
        {
            emulation.ExternalsManager.AddExternal(
                new AP_UnixSocketTerminal(path), name);
        }
    }

    [Transient]
    public class AP_UnixSocketTerminal : BackendTerminal, IDisposable
    {
        public AP_UnixSocketTerminal(string path)
        {
            if(String.IsNullOrWhiteSpace(path) || !Path.IsPathRooted(path))
            {
                throw new RecoverableException(
                    "Unix terminal path must be absolute");
            }
            outgoing = new ConcurrentQueue<byte>();
            outgoingReady = new AutoResetEvent(false);
            listener = new Socket(AddressFamily.Unix, SocketType.Stream,
                ProtocolType.Unspecified);
            try
            {
                listener.Bind(new UnixDomainSocketEndPoint(path));
                listener.Listen(1);
            }
            catch(Exception error)
            {
                listener.Dispose();
                throw new RecoverableException(String.Format(
                    "Failed to listen on Unix socket {0}: {1}",
                    path, error.Message));
            }
            listenerThread = new Thread(Listen)
            {
                IsBackground = true,
                Name = GetType().Name
            };
            listenerThread.Start();
        }

        public override void WriteChar(byte value)
        {
            outgoing.Enqueue(value);
            outgoingReady.Set();
        }

        public void Dispose()
        {
            stopRequested = true;
            listener.Close();
            CloseClient();
            outgoingReady.Set();
            if(Thread.CurrentThread != listenerThread)
            {
                listenerThread.Join();
            }
            outgoingReady.Dispose();
        }

        private void Listen()
        {
            while(!stopRequested)
            {
                Socket connection;
                try
                {
                    connection = listener.Accept();
                }
                catch(ObjectDisposedException)
                {
                    break;
                }
                catch(SocketException)
                {
                    if(stopRequested)
                    {
                        break;
                    }
                    continue;
                }

                lock(clientLock)
                {
                    client = connection;
                }
                var writer = new Thread(() => Write(connection))
                {
                    IsBackground = true,
                    Name = GetType().Name + "_Writer"
                };
                writer.Start();
                Read(connection);
                CloseClient(connection);
                outgoingReady.Set();
                writer.Join();
            }
        }

        private void Read(Socket connection)
        {
            var buffer = new byte[4096];
            try
            {
                while(!stopRequested && IsCurrent(connection))
                {
                    var count = connection.Receive(buffer);
                    if(count == 0)
                    {
                        break;
                    }
                    for(var index = 0; index < count; index++)
                    {
                        CallCharReceived(buffer[index]);
                    }
                }
            }
            catch(ObjectDisposedException)
            {
            }
            catch(SocketException)
            {
            }
        }

        private void Write(Socket connection)
        {
            var buffer = new byte[4096];
            try
            {
                while(!stopRequested && IsCurrent(connection))
                {
                    if(!outgoing.TryDequeue(out buffer[0]))
                    {
                        outgoingReady.WaitOne();
                        continue;
                    }
                    var count = 1;
                    while(count < buffer.Length &&
                          outgoing.TryDequeue(out buffer[count]))
                    {
                        count++;
                    }
                    var offset = 0;
                    while(offset < count)
                    {
                        var sent = connection.Send(
                            buffer, offset, count - offset, SocketFlags.None);
                        if(sent == 0)
                        {
                            CloseClient(connection);
                            return;
                        }
                        offset += sent;
                    }
                }
            }
            catch(ObjectDisposedException)
            {
            }
            catch(SocketException)
            {
                CloseClient(connection);
            }
        }

        private bool IsCurrent(Socket connection)
        {
            lock(clientLock)
            {
                return client == connection;
            }
        }

        private void CloseClient(Socket connection = null)
        {
            Socket closing;
            lock(clientLock)
            {
                if(connection != null && client != connection)
                {
                    return;
                }
                closing = client;
                client = null;
            }
            if(closing == null)
            {
                return;
            }
            try
            {
                closing.Shutdown(SocketShutdown.Both);
            }
            catch(SocketException)
            {
            }
            catch(ObjectDisposedException)
            {
            }
            closing.Close();
        }

        private volatile bool stopRequested;
        private Socket client;

        private readonly Socket listener;
        private readonly Thread listenerThread;
        private readonly ConcurrentQueue<byte> outgoing;
        private readonly AutoResetEvent outgoingReady;
        private readonly object clientLock = new object();
    }
}
