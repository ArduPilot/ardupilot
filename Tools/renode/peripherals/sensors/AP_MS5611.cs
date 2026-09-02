// MS5611 barometer on SPI. The PROM contains the calibration example from
// the datasheet (with a valid CRC), and conversions return the corresponding
// sea-level pressure and room-temperature samples.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_MS5611 : ISPIPeripheral, II2CPeripheral, IGPIOReceiver
    {
        public AP_MS5611()
        {
            prom = new ushort[] { 0, 40127, 36924, 23317, 23282, 33464, 28312, 0 };
            Reset();
        }

        public void Reset()
        {
            command = 0;
            transferByte = 0;
            conversion = TemperatureConversion;
        }

        public byte Transmit(byte value)
        {
            if(transferByte++ == 0)
            {
                SetCommand(value);
                return 0;
            }
            return ReadResponse(transferByte - 2);
        }

        public void Write(byte[] data)
        {
            if(data.Length > 0)
            {
                SetCommand(data[0]);
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var index = 0; index < count; index++)
            {
                result[index] = ReadResponse(index);
            }
            return result;
        }

        private void SetCommand(byte value)
        {
            command = value;
            if((command & ConversionMask) == PressureConversion)
            {
                conversion = PressureConversion;
            }
            else if((command & ConversionMask) == TemperatureConversion)
            {
                conversion = TemperatureConversion;
            }
        }

        private byte ReadResponse(int index)
        {
            if(command == ReadAdc)
            {
                var sample = conversion == PressureConversion ? PressureSample : TemperatureSample;
                var shift = 8 * (2 - index);
                return shift >= 0 ? (byte)(sample >> shift) : (byte)0;
            }

            if(command >= PromBase && command <= PromEnd && (command & 1) == 0)
            {
                var valueIndex = (command - PromBase) / 2;
                var shift = 8 * (1 - index);
                return shift >= 0 ? (byte)(prom[valueIndex] >> shift) : (byte)0;
            }

            return 0;
        }

        public void FinishTransmission()
        {
            transferByte = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                transferByte = 0;
            }
        }

        private readonly ushort[] prom;
        private byte command;
        private byte conversion;
        private int transferByte;

        private const byte ReadAdc = 0x00;
        private const byte PressureConversion = 0x40;
        private const byte TemperatureConversion = 0x50;
        private const byte ConversionMask = 0xF0;
        private const byte PromBase = 0xA0;
        private const byte PromEnd = 0xAE;
        private const uint PressureSample = 9085466;
        private const uint TemperatureSample = 8569150;
    }
}
