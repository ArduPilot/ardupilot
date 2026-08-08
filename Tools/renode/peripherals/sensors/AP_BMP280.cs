//
// BMP280 barometer as an I2C register-pointer device. Serves what
// AP_Baro_BMP280 needs: ID, the 24-byte calibration block, read-back
// CTRL_MEAS/CONFIG (checked registers, re-read every 20 transfers) and
// the 6-byte pressure/temperature data block. Calibration constants
// and raw readings are the BMP280 datasheet worked example, which the
// driver's compensation turns into ~100653 Pa / ~25.1 C - a stable,
// plausible bench condition.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_BMP280 : II2CPeripheral
    {
        public AP_BMP280()
        {
            Reset();
        }

        public void Reset()
        {
            pointer = 0;
            registers = new byte[256];

            registers[ID] = 0x58;

            // dig_T1..dig_P9, little-endian, datasheet worked example
            WriteU16(0x88, 27504);
            WriteS16(0x8A, 26435);
            WriteS16(0x8C, -1000);
            WriteU16(0x8E, 36477);
            WriteS16(0x90, -10685);
            WriteS16(0x92, 3024);
            WriteS16(0x94, 2855);
            WriteS16(0x96, 140);
            WriteS16(0x98, -7);
            WriteS16(0x9A, 15500);
            WriteS16(0x9C, -14600);
            WriteS16(0x9E, 6000);

            // adc_P = 415148 (0x655CC), adc_T = 519888 (0x7EED0):
            // 20-bit values as msb / lsb / xlsb<<4
            registers[0xF7] = 0x65;
            registers[0xF8] = 0x5C;
            registers[0xF9] = 0xC0;
            registers[0xFA] = 0x7E;
            registers[0xFB] = 0xED;
            registers[0xFC] = 0x00;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            pointer = data[0];
            // register writes arrive as (reg, value) pairs
            for(var i = 1; i < data.Length; i += 2)
            {
                switch(pointer)
                {
                case RESET:
                    if(data[i] == 0xB6)
                    {
                        var saved = registers;
                        Reset();
                        registers = saved; // measurement/cal content survives; config resets
                        registers[CTRL_MEAS] = 0;
                        registers[CONFIG] = 0;
                    }
                    break;
                case STATUS:
                    break; // read-only
                default:
                    registers[pointer] = data[i];
                    break;
                }
                if(i + 1 < data.Length)
                {
                    pointer = data[i + 1];
                }
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = registers[pointer];
                pointer = (pointer + 1) & 0xFF;
            }
            return result;
        }

        public void FinishTransmission()
        {
        }

        private void WriteU16(int offset, ushort value)
        {
            registers[offset] = (byte)(value & 0xFF);
            registers[offset + 1] = (byte)(value >> 8);
        }

        private void WriteS16(int offset, short value)
        {
            WriteU16(offset, (ushort)value);
        }

        private byte[] registers;
        private int pointer;

        private const int ID = 0xD0;
        private const int RESET = 0xE0;
        private const int STATUS = 0xF3;
        private const int CTRL_MEAS = 0xF4;
        private const int CONFIG = 0xF5;
    }
}
