using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.IO;

namespace ArdupilotMega.Comms
{
    public class CommsFile : ICommsSerial
    {
        // Methods
        public void Close() { BaseStream.Close(); }
        public void DiscardInBuffer() { }
        //void DiscardOutBuffer();
        public void Open()
        {
            BaseStream = File.OpenRead(PortName);
        }
        public int Read(byte[] buffer, int offset, int count)
        {
            return BaseStream.Read(buffer, offset, count);
        }
        //int Read(char[] buffer, int offset, int count);
        public int ReadByte() { return BaseStream.ReadByte(); }
        public int ReadChar() { return BaseStream.ReadByte(); }
        public string ReadExisting() { return ""; }
        public string ReadLine() { return ""; }
        //string ReadTo(string value);
        public void Write(string text) { }
        public void Write(byte[] buffer, int offset, int count) { }
        //void Write(char[] buffer, int offset, int count);
        public void WriteLine(string text) { }

        public void toggleDTR() { }

        // Properties
        public Stream BaseStream { get; private set; }
        public int BaudRate { get; set; }
        public int BytesToRead { get { return (int)(BaseStream.Length - BaseStream.Position); } }
        public int BytesToWrite { get; set; }
        public int DataBits  { get; set; }
        public bool DtrEnable { get; set; }
        public bool IsOpen { get { return (BaseStream != null); } }

        public Parity Parity { get; set; }

        public string PortName { get; set; }
        public int ReadBufferSize { get; set; }
        public int ReadTimeout { get; set; }
        public bool RtsEnable { get; set; }
        public StopBits StopBits { get; set; }
        public int WriteBufferSize { get; set; }
        public int WriteTimeout { get; set; }
    }
}
