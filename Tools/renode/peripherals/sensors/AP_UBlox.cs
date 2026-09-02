// Minimal u-blox receiver for AP_Periph GPS testing. It emits a stationary
// 3D fix using the UBX NAV-PVT and NAV-TIMEGPS messages understood by the
// reduced AP_Periph GPS build.
using System;
using System.Collections.Generic;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_UBlox : IUART, IDoubleWordPeripheral, IKnownSize
    {
        public AP_UBlox(IMachine machine)
        {
            this.machine = machine;
            transmitter = machine.ObtainManagedThread(
                SendNavigation, MessagesPerSecond, name: "AP u-blox navigation", owner: this);
            transmitter.Start();
        }

        public void Reset()
        {
            generation++;
            itow = InitialTimeOfWeekMs;
        }

        public void WriteChar(byte value)
        {
            // Configuration sent by the firmware is intentionally accepted
            // without changing the deterministic navigation stream.
        }

        public uint BaudRate => 230400;
        public Parity ParityBit => Parity.None;
        public Bits StopBits => Bits.One;
        public long Size => 4;

        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        [field: Transient]
        public event Action<byte> CharReceived;

        private void SendNavigation()
        {
            itow += 1000U / MessagesPerSecond;
            var output = new List<byte>();

            var pvt = new List<byte>();
            AddUInt32(pvt, itow);
            AddUInt16(pvt, 2026);
            pvt.AddRange(new byte[] { 8, 8, 12, 0, 0, 0x07 });
            AddUInt32(pvt, 10000);       // UTC time accuracy, ns
            AddInt32(pvt, 0);            // fractional UTC time, ns
            pvt.AddRange(new byte[] { 3, 1, 0, 15 }); // 3D fix, valid, satellites
            AddInt32(pvt, 1491652300);   // longitude, 1e-7 degrees
            AddInt32(pvt, -353632610);   // latitude, 1e-7 degrees
            AddInt32(pvt, 600000);       // ellipsoid height, mm
            AddInt32(pvt, 584000);       // MSL height, mm
            AddUInt32(pvt, 500);         // horizontal accuracy, mm
            AddUInt32(pvt, 800);         // vertical accuracy, mm
            AddInt32(pvt, 0);            // north velocity, mm/s
            AddInt32(pvt, 0);            // east velocity, mm/s
            AddInt32(pvt, 0);            // down velocity, mm/s
            AddInt32(pvt, 0);            // ground speed, mm/s
            AddInt32(pvt, 0);            // course, 1e-5 degrees
            AddUInt32(pvt, 100);         // speed accuracy, mm/s
            AddUInt32(pvt, 10000);       // heading accuracy, 1e-5 degrees
            AddUInt16(pvt, 120);         // position DOP, 0.01
            pvt.AddRange(new byte[6]);    // flags3 and reserved1
            AddInt32(pvt, 0);            // vehicle heading
            AddInt16(pvt, 0);            // magnetic declination
            AddUInt16(pvt, 0);           // magnetic accuracy
            AppendFrame(output, 0x01, 0x07, pvt);

            var timeGps = new List<byte>();
            AddUInt32(timeGps, itow);
            AddInt32(timeGps, 0);
            AddUInt16(timeGps, GpsWeek);
            timeGps.Add(18);             // leap seconds
            timeGps.Add(0x07);           // TOW, week and leap seconds valid
            AddUInt32(timeGps, 10000);
            AppendFrame(output, 0x01, 0x20, timeGps);
            ScheduleOutput(output);
        }

        private void AppendFrame(List<byte> output, byte messageClass,
            byte messageId, List<byte> payload)
        {
            var body = new List<byte> {
                messageClass, messageId,
                (byte)payload.Count, (byte)(payload.Count >> 8),
            };
            body.AddRange(payload);
            byte checksumA = 0;
            byte checksumB = 0;
            foreach(var value in body)
            {
                checksumA += value;
                checksumB += checksumA;
            }
            output.Add(0xB5);
            output.Add(0x62);
            output.AddRange(body);
            output.Add(checksumA);
            output.Add(checksumB);
        }

        private void ScheduleOutput(List<byte> output)
        {
            var scheduledGeneration = generation;
            for(var i = 0; i < output.Count; i++)
            {
                var value = output[i];
                machine.ScheduleAction(
                    TimeInterval.FromMicroseconds((ulong)i * InterByteDelayUs), _ =>
                    {
                        if(scheduledGeneration != generation)
                        {
                            return;
                        }
                        CharReceived?.Invoke(value);
                    }, name: "AP u-blox serial output");
            }
        }

        private static void AddInt16(List<byte> output, short value) =>
            AddUInt16(output, unchecked((ushort)value));
        private static void AddUInt16(List<byte> output, ushort value)
        {
            output.Add((byte)value);
            output.Add((byte)(value >> 8));
        }
        private static void AddInt32(List<byte> output, int value) =>
            AddUInt32(output, unchecked((uint)value));
        private static void AddUInt32(List<byte> output, uint value)
        {
            output.Add((byte)value);
            output.Add((byte)(value >> 8));
            output.Add((byte)(value >> 16));
            output.Add((byte)(value >> 24));
        }

        private readonly IMachine machine;
        private readonly IManagedThread transmitter;
        private uint generation;
        private uint itow = InitialTimeOfWeekMs;

        private const uint MessagesPerSecond = 5;
        private const uint InterByteDelayUs = 1000;
        private const uint InitialTimeOfWeekMs = 518400000;
        private const ushort GpsWeek = 2430;
    }
}
