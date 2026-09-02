// IR-LOCK I2C precision-landing camera stream.
using System;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_IRLock : II2CPeripheral
    {
        public AP_IRLock()
        {
            Reset();
        }

        public ushort PixelX { get; set; }
        public ushort PixelY { get; set; }
        public ushort PixelSizeX { get; set; }
        public ushort PixelSizeY { get; set; }
        public bool CorruptChecksum { get; set; }
        public bool SuppressFrames { get; set; }
        public uint FrameCount { get; private set; }

        public void Reset()
        {
            PixelX = 120;
            PixelY = 70;
            PixelSizeX = 20;
            PixelSizeY = 10;
            CorruptChecksum = false;
            SuppressFrames = false;
            FrameCount = 0;
            offset = 0;
            frame = null;
        }

        public void Write(byte[] data)
        {
            // The camera is a byte stream; writes do not select registers.
        }

        public byte[] Read(int count = 1)
        {
            if(SuppressFrames)
            {
                return new byte[count];
            }

            if(frame == null || offset >= frame.Length)
            {
                frame = BuildFrame();
                offset = 0;
            }

            var result = new byte[count];
            var copied = 0;
            while(copied < count)
            {
                var available = frame.Length - offset;
                var chunk = Math.Min(available, count - copied);
                Array.Copy(frame, offset, result, copied, chunk);
                copied += chunk;
                offset += chunk;
                if(offset == frame.Length && copied < count)
                {
                    frame = BuildFrame();
                    offset = 0;
                }
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private byte[] BuildFrame()
        {
            FrameCount++;
            var result = new byte[FrameSize];
            PutUInt32(result, 0, SyncWord);
            var checksum = (ushort)(Signature + PixelX + PixelY +
                PixelSizeX + PixelSizeY);
            if(CorruptChecksum)
            {
                checksum++;
            }
            PutUInt16(result, 4, checksum);
            PutUInt16(result, 6, Signature);
            PutUInt16(result, 8, PixelX);
            PutUInt16(result, 10, PixelY);
            PutUInt16(result, 12, PixelSizeX);
            PutUInt16(result, 14, PixelSizeY);
            return result;
        }

        private static void PutUInt16(byte[] buffer, int offset, ushort value)
        {
            buffer[offset] = (byte)(value & 0xFF);
            buffer[offset + 1] = (byte)(value >> 8);
        }

        private static void PutUInt32(byte[] buffer, int offset, uint value)
        {
            buffer[offset] = (byte)(value & 0xFF);
            buffer[offset + 1] = (byte)((value >> 8) & 0xFF);
            buffer[offset + 2] = (byte)((value >> 16) & 0xFF);
            buffer[offset + 3] = (byte)((value >> 24) & 0xFF);
        }

        private byte[] frame;
        private int offset;

        private const uint SyncWord = 0xAA55AA55;
        private const ushort Signature = 1;
        private const int FrameSize = 16;
    }
}
