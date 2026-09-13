// Physics-driven BMM150 compass model for the ArduPilot I2C backend.
//
// This is the part mr_vmu_rt1176 DECLARES in its hwdef, at I2C bus 2 address
// 0x10. Modelling it is what lets that board find a compass under Renode
// without touching firmware. The alternative - naming another compass driver
// in the board's hwdef so an emulated IST8310 could be probed - was measured
// on silicon and costs 315 Hz of main loop rate down to 124 Hz, because the
// extra driver retries a device the real board does not have. See the warning
// in hwdef.dat.
//
// AP_Compass_BMM150 applies Bosch's compensation algorithm to every sample,
// so the model cannot simply publish a field: it has to publish raw counts
// that come back out of that algorithm as the field we want. The trim values
// below are chosen to make the compensation an identity, which is what makes
// that inversion one multiply instead of a numerical solve:
//
//   xyz1 = rhall            -> the `inter` term collapses to zero, so
//   x1 = y1 = 0                _compensate_xy() reduces to
//   x2 = y2 = -128             val = (x2 + 0xA0) * xy / 32 = xy
//   xy1 = xy2 = 0
//
//   z1 = 0, z2 = 16384      -> divisor is 16384 and the z3 term drops out
//   z3 = 0, z4 = 0             with rhall == xyz1, so _compensate_z()
//                              reduces to (z << 15) / 16384 = 2z
//
// The driver then divides by 16 (16 LSB/uT) and multiplies by 10 to reach
// milligauss, so a compensated count is 1.6 per milligauss. X and Y are
// published shifted left 3 and Z shifted left 1, because the driver shifts
// them back down by exactly that much.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_BMM150 : AP_I2CRegisterDevice
    {
        public AP_BMM150(IMachine machine, byte rotation = 0)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Rotation = rotation;
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            Registers[ChipIdReg] = ChipIdValue;
            WriteTrim();
            // Something valid before the first physics read, so a probe that
            // lands between resets does not see an all-zero field.
            PublishField(0.0f, 0.0f, 0.0f);
        }

        public override byte[] Read(int count = 1)
        {
            UpdateSample();
            return base.Read(count);
        }

        protected override void WriteRegister(int register, byte value)
        {
            // A soft reset must not wipe the chip ID or the trim: the driver
            // issues one, then immediately reads both back.
            if(register == PowerAndOperationsReg && (value & SoftReset) != 0)
            {
                Reset();
                return;
            }
            base.WriteRegister(register, value);
        }

        private void UpdateSample()
        {
            var field = AP_SensorOrientation.BodyToSensor(
                physics.Current.MagneticFieldBodyMgauss, Rotation);
            PublishField(field[0], field[1], field[2]);
        }

        private void PublishField(float xMgauss, float yMgauss, float zMgauss)
        {
            // Raw counts the driver's compensation turns back into this field.
            WriteS16LE(DataXLsbReg + 0, (short)(Compensated(xMgauss) << 3));
            WriteS16LE(DataXLsbReg + 2, (short)(Compensated(yMgauss) << 3));
            WriteS16LE(DataXLsbReg + 4, (short)(Compensated(zMgauss) << 1));
            // Hall word: rhall in bits 15:2, and bit 0 is the data-ready flag
            // the driver refuses samples without.
            WriteU16LE(DataXLsbReg + 6, (ushort)((RHall << 2) | 1));
        }

        private static int Compensated(float milligauss)
        {
            var counts = Math.Round(milligauss * CountsPerMilligauss);
            // Leave headroom for the shift the driver undoes.
            return (int)Math.Max(-4095, Math.Min(4095, counts));
        }

        private void WriteTrim()
        {
            Registers[DigX1Reg] = 0;                       // x1
            Registers[DigX1Reg + 1] = 0;                   // y1
            WriteS16LE(DigZ4LsbReg, 0);                    // z4
            Registers[DigX2Reg] = unchecked((byte)-128);   // x2
            Registers[DigX2Reg + 1] = unchecked((byte)-128);  // y2
            WriteS16LE(DigZ2LsbReg, 16384);                // z2
            WriteS16LE(DigZ1LsbReg, 0);                    // z1
            WriteU16LE(DigXyz1LsbReg, RHall);              // xyz1 == rhall
            WriteS16LE(DigZ3LsbReg, 0);                    // z3
            Registers[DigXy2Reg] = 0;                      // xy2
            Registers[DigXy2Reg + 1] = 0;                  // xy1
        }

        private readonly AP_PhysicsState physics;
        public byte Rotation { get; set; }

        private const int ChipIdReg = 0x40;
        private const byte ChipIdValue = 0x32;
        private const int DataXLsbReg = 0x42;
        private const int PowerAndOperationsReg = 0x4B;
        private const byte SoftReset = (1 << 7) | (1 << 1);
        private const int DigX1Reg = 0x5D;
        private const int DigZ4LsbReg = 0x62;
        private const int DigX2Reg = 0x64;
        private const int DigZ2LsbReg = 0x68;
        private const int DigZ1LsbReg = 0x6A;
        private const int DigXyz1LsbReg = 0x6C;
        private const int DigZ3LsbReg = 0x6E;
        private const int DigXy2Reg = 0x70;
        private const ushort RHall = 8192;
        private const double CountsPerMilligauss = 1.6;
    }
}
