// STM32 I2Cv2 model with DMA requests for ChibiOS.  Renode's stock
// STM32F7_I2C implements the transfer state machine but treats TXDMAEN and
// RXDMAEN as tags.  This wrapper retains those bits and turns TXIS/RXNE into
// request pulses while delegating all register and child-device behavior.
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class AP_STM32_I2C : SimpleContainer<II2CPeripheral>,
        II2CPeripheral, IBytePeripheral, IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32_I2C(IMachine machine) : base(machine)
        {
            inner = new STM32F7_I2C(machine);
            machine.RegisterAsAChildOf(
                machine.SystemBus, inner, NullRegistrationPoint.Instance);
            TxDmaRequest = new GPIO();
            RxDmaRequest = new GPIO();
            poller = machine.ObtainManagedThread(
                Poll, PollFrequency, name: "AP STM32 I2C DMA", owner: this);
            poller.Start();
        }

        public uint ReadDoubleWord(long offset)
        {
            return offset == Control1 ? inner.ReadDoubleWord(offset) | dmaEnableBits
                                      : inner.ReadDoubleWord(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == Control1)
            {
                dmaEnableBits = value & (TxDmaEnabled | RxDmaEnabled);
                value &= ~(TxDmaEnabled | RxDmaEnabled);
            }
            else if(offset == Timing)
            {
                // Timing does not affect the transfer-level model.  The
                // stock peripheral tags these fields as unimplemented.
                return;
            }
            else if(offset == InterruptClear)
            {
                // ChibiOS clears TCCF, which is reserved in the stock model.
                value &= ~TransferCompleteClear;
            }
            inner.WriteDoubleWord(offset, value);
            if(offset == Control2)
            {
                ServiceDma(MaximumTransferLength);
            }
        }

        public byte ReadByte(long offset)
        {
            return (byte)ReadDoubleWord(offset);
        }

        public void WriteByte(long offset, byte value)
        {
            WriteDoubleWord(offset, value);
        }

        public override void Reset()
        {
            dmaEnableBits = 0;
            TxRequests = 0;
            RxRequests = 0;
            inner.Reset();
            TxDmaRequest.Unset();
            RxDmaRequest.Unset();
        }

        public void Write(byte[] data) => inner.Write(data);
        public byte[] Read(int count = 1) => inner.Read(count);
        public void FinishTransmission() => inner.FinishTransmission();

        public override void Register(II2CPeripheral peripheral,
            NumberRegistrationPoint<int> registrationPoint)
        {
            inner.Register(peripheral, registrationPoint);
        }

        public override void Unregister(II2CPeripheral peripheral)
        {
            inner.Unregister(peripheral);
        }

        public override IEnumerable<NumberRegistrationPoint<int>>
            GetRegistrationPoints(II2CPeripheral peripheral)
        {
            return inner.GetRegistrationPoints(peripheral);
        }

        public override IEnumerable<IRegistered<II2CPeripheral,
            NumberRegistrationPoint<int>>> Children => inner.Children;

        public override void Dispose()
        {
            inner.Dispose();
        }

        public GPIO EventInterrupt => inner.EventInterrupt;
        public GPIO ErrorInterrupt => inner.ErrorInterrupt;
        public GPIO TxDmaRequest { get; }
        public GPIO RxDmaRequest { get; }
        public bool RxNotEmpty => inner.RxNotEmpty;
        public bool OwnAddress1Enabled => inner.OwnAddress1Enabled;
        public long Size => inner.Size;

        private void Poll()
        {
            ServiceDma(1);
        }

        private void ServiceDma(int maximumRequests)
        {
            // ChibiOS starts short transfers with a timeout shorter than a
            // managed-thread scheduling interval can reliably provide. Drain
            // requests synchronously when START is written; Poll handles any
            // request which becomes ready later.
            for(var i = 0; i < maximumRequests; i++)
            {
                var status = inner.ReadDoubleWord(Status);
                if((dmaEnableBits & RxDmaEnabled) != 0 &&
                    (status & RxNotEmptyFlag) != 0)
                {
                    RxRequests++;
                    RxDmaRequest.Blink();
                    continue;
                }
                if((dmaEnableBits & TxDmaEnabled) != 0 &&
                    (status & TxInterruptStatus) != 0)
                {
                    TxRequests++;
                    TxDmaRequest.Blink();
                    continue;
                }
                break;
            }
        }

        private readonly STM32F7_I2C inner;
        private readonly IManagedThread poller;
        private uint dmaEnableBits;

        public ulong TxRequests { get; private set; }
        public ulong RxRequests { get; private set; }

        private const uint PollFrequency = 10000;
        private const long Control1 = 0x00;
        private const long Control2 = 0x04;
        private const long Timing = 0x10;
        private const long Status = 0x18;
        private const long InterruptClear = 0x1C;
        private const uint TxDmaEnabled = 1U << 14;
        private const uint RxDmaEnabled = 1U << 15;
        private const uint TransferCompleteClear = 1U << 6;
        private const uint TxInterruptStatus = 1U << 1;
        private const uint RxNotEmptyFlag = 1U << 2;
        private const int MaximumTransferLength = 255;
    }
}
