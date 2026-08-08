// Deterministic differential-pressure sensor shared by the MS4525 and DLVR
// AP_Airspeed backends.  Both devices use the same four-byte pressure and
// temperature layout at address 0x28; writes trigger a conversion on MS4525
// and are harmless for DLVR.
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_Airspeed : II2CPeripheral
    {
        public AP_Airspeed()
        {
            Reset();
        }

        public void Reset()
        {
            // Mid-scale pressure is zero differential pressure.  768 maps to
            // approximately 25 C in both firmware backends.
            pressure = 8192;
            temperature = 768;
            readIndex = 0;
            readSinceFinish = false;
        }

        public void Write(byte[] data)
        {
            readIndex = 0;
        }

        public byte[] Read(int count = 1)
        {
            var packed = ((uint)(pressure & 0x3FFF) << 16)
                | ((uint)(temperature & 0x7FF) << 5);
            var sample = new byte[] {
                (byte)(packed >> 24), (byte)(packed >> 16),
                (byte)(packed >> 8), (byte)packed,
            };
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = sample[(readIndex + i) % sample.Length];
            }
            readIndex = (readIndex + count) % sample.Length;
            readSinceFinish = true;
            return result;
        }

        public void FinishTransmission()
        {
            if(readSinceFinish)
            {
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        private ushort pressure;
        private ushort temperature;
        private int readIndex;
        private bool readSinceFinish;
    }

    // AUAV differential pressure sensor at 0x26. Zeroed compensation
    // coefficients and a midpoint pressure sample produce 0 Pa at 25 C.
    public class AP_Airspeed_AUAV : II2CPeripheral
    {
        public AP_Airspeed_AUAV()
        {
            Reset();
        }

        public void Reset()
        {
            command = 0xAE;
            readIndex = 0;
            readSinceFinish = false;
        }

        public void Write(byte[] data)
        {
            if(data.Length != 0)
            {
                command = data[0];
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        public byte[] Read(int count = 1)
        {
            byte[] sample;
            var status = Absolute ? (byte)0x40 : (byte)0x50;
            if(command >= 0x2B && command <= 0x38)
            {
                sample = new byte[] { status, 0, 0 };
            }
            else if(Absolute)
            {
                sample = new byte[] {
                    status, 0xB5, 0xE9, 0xE2, 0x73, 0x9C, 0xE7,
                };
            }
            else
            {
                sample = new byte[] {
                    status, 0x80, 0, 0, 0x73, 0x9C, 0xE7,
                };
            }
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = sample[(readIndex + i) % sample.Length];
            }
            readIndex = (readIndex + count) % sample.Length;
            readSinceFinish = true;
            return result;
        }

        public void FinishTransmission()
        {
            if(readSinceFinish)
            {
                readIndex = 0;
                readSinceFinish = false;
            }
        }

        protected bool Absolute;
        private byte command;
        private int readIndex;
        private bool readSinceFinish;
    }

    public class AP_Baro_AUAV : AP_Airspeed_AUAV
    {
        public AP_Baro_AUAV()
        {
            Absolute = true;
        }
    }

    // QioTek ASP5033 register interface at 0x6C. It reports a ready,
    // stationary 0 Pa sample at 25 C and implements the mutable WHOAMI probe.
    public class AP_Airspeed_ASP5033 : II2CPeripheral
    {
        public AP_Airspeed_ASP5033()
        {
            Reset();
        }
        public void Reset()
        {
            register = 0;
            identity = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            register = data[0];
            if(data.Length > 1 && register == IdentitySet)
            {
                identity = data[1];
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                var address = register + i;
                if(address == Identity || address == IdentitySet)
                {
                    result[i] = identity;
                }
                else if(address == Command)
                {
                    result[i] = Ready;
                }
                else if(address == Temperature)
                {
                    result[i] = 25;
                }
            }
            register += (byte)count;
            return result;
        }

        public void FinishTransmission()
        {
        }

        private byte register;
        private byte identity;

        private const byte Identity = 0x01;
        private const byte Temperature = 0x09;
        private const byte Command = 0x30;
        private const byte IdentitySet = 0xA4;
        private const byte Ready = 0x08;
    }
}
