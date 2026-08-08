//
// STM32 F4 I2C (I2Cv1) master model for ChibiOS, forked from Renode's
// STM32F4_I2C (MIT). The stock model cannot run ChibiOS's driver:
//
//  - ChibiOS I2Cv1 is DMA-only on the F4; the stock model has no DMA
//    request output at all, so master receive never moves a byte.
//  - The event ISR decodes (SR1 | SR2<<16) against exact patterns.
//    EV5/EV6/EV8_2 all require SR2.BUSY, which the stock model never
//    sets; EV6_MASTER_REC_MODE_SELECTED requires SR1 to be exactly
//    ADDR, but the stock model also raises BTF at a read address
//    match. Either way the decode falls through to default and the
//    transaction dies at the address phase.
//
// Master receive works with the stock STM32DMA like this: the ChibiOS
// ADDR handler enables the RX stream and then sets CR2.LAST, so the
// LAST write is an order-correct "stream is armed" signal. From it we
// pump RxDmaRequest blinks (a small burst per scheduled action); each
// blink makes the DMA read DR, which lazily fetches the next byte from
// the addressed slave. When NDTR hits zero the DMA completion ISR
// issues STOP, which stops the pump.
//
// The stock stream DMA drains memory-to-peripheral transfers when the
// stream is enabled. The F1 channel DMA instead uses TxDmaRequest,
// asserted after the write address is accepted. In both cases the
// bytes arrive as back-to-back DR writes; the last one leaves BTF|TXE
// set for the EV8_2 decode once the DMA-complete handler re-enables
// ITEVTEN.
//
using System;
using System.Collections.Generic;
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.I2C
{
    [AllowedTranslations(AllowedTranslation.WordToDoubleWord)]
    public sealed class AP_STM32F4_I2C : SimpleContainer<II2CPeripheral>, IDoubleWordPeripheral, IBytePeripheral, IKnownSize
    {
        public AP_STM32F4_I2C(IMachine machine) : base(machine)
        {
            EventInterrupt = new GPIO();
            ErrorInterrupt = new GPIO();
            RxDmaRequest = new GPIO();
            TxDmaRequest = new GPIO();
            CreateRegisters();
            Reset();
        }

        public byte ReadByte(long offset)
        {
            if((Registers)offset == Registers.Data)
            {
                return (byte)DataRead();
            }
            this.LogUnhandledRead(offset);
            return 0;
        }

        public void WriteByte(long offset, byte value)
        {
            if((Registers)offset == Registers.Data)
            {
                DataWrite(value);
            }
            else
            {
                this.LogUnhandledWrite(offset, value);
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            return registers.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            registers.Write(offset, value);
        }

        public override void Reset()
        {
            state = State.Idle;
            busy = false;
            pumpActive = false;
            addrClearArmed = false;
            dataToTransfer = null;
            dataToReceive = null;
            selectedSlave = null;
            EventInterrupt.Unset();
            ErrorInterrupt.Unset();
            RxDmaRequest.Unset();
            TxDmaRequest.Unset();
            registers.Reset();
        }

        public GPIO EventInterrupt { get; }
        public GPIO ErrorInterrupt { get; }
        public GPIO RxDmaRequest { get; }
        public GPIO TxDmaRequest { get; }

        public long Size => 0x400;

        private void CreateRegisters()
        {
            var control1 = new DoubleWordRegister(this)
                .WithFlag(15, writeCallback: SoftwareResetWrite, name: "SWRST")
                .WithFlag(9, FieldMode.Read, writeCallback: StopWrite, name: "StopGen")
                .WithFlag(8, FieldMode.Read, writeCallback: StartWrite, name: "StartGen")
                .WithFlag(0, writeCallback: PeripheralEnableWrite, name: "PeriEn");
            acknowledgeEnable = control1.DefineFlagField(10);

            var control2 = new DoubleWordRegister(this)
                .WithValueField(0, 6, name: "Freq");
            bufferInterruptEnable = control2.DefineFlagField(10, changeCallback: InterruptEnableChange);
            eventInterruptEnable = control2.DefineFlagField(9, changeCallback: InterruptEnableChange);
            errorInterruptEnable = control2.DefineFlagField(8);
            // DMAEN: stored, not acted on - the LAST write is the RX arm signal
            dmaEnable = control2.DefineFlagField(11);
            lastDma = control2.DefineFlagField(12, changeCallback: (_, newValue) =>
            {
                if(newValue && state == State.Receiving)
                {
                    StartRxPump();
                }
            });

            var status1 = new DoubleWordRegister(this);
            acknowledgeFailed = status1.DefineFlagField(10, FieldMode.ReadToClear | FieldMode.WriteZeroToClear, changeCallback: (_, __) => Update());
            dataRegisterEmpty = status1.DefineFlagField(7, FieldMode.Read);
            dataRegisterNotEmpty = status1.DefineFlagField(6, FieldMode.Read, valueProviderCallback: _ => dataToReceive?.Any() ?? false);
            byteTransferFinished = status1.DefineFlagField(2, FieldMode.Read);
            addressSentOrMatched = status1.DefineFlagField(1, FieldMode.Read);
            startBit = status1.DefineFlagField(0, FieldMode.Read);
            // ADDR is cleared by an SR1 read followed by an SR2 read, in
            // that order. The ChibiOS ISR reads SR2 first and SR1 second,
            // so clearing on any SR2 read (as the stock model does) wipes
            // ADDR before the ISR has decoded the event.
            status1.WithReadCallback((_, __) =>
            {
                if(addressSentOrMatched.Value)
                {
                    addrClearArmed = true;
                }
            });

            var status2 = new DoubleWordRegister(this);
            transmitterReceiver = status2.DefineFlagField(2, FieldMode.Read);
            busyFlag = status2.DefineFlagField(1, FieldMode.Read, valueProviderCallback: _ => busy);
            masterSlave = status2.DefineFlagField(0, FieldMode.Read, readCallback: (_, __) =>
            {
                if(addrClearArmed)
                {
                    addrClearArmed = false;
                    addressSentOrMatched.Value = false;
                    Update();
                }
            });

            var data = new DoubleWordRegister(this);
            data.DefineValueField(0, 8, valueProviderCallback: _ => DataRead(), writeCallback: (_, val) => DataWrite((uint)val));

            var registerDictionary = new Dictionary<long, DoubleWordRegister>
            {
                {(long)Registers.RiseTime, DoubleWordRegister.CreateRWRegister(0x2)},
                {(long)Registers.ClockControl, DoubleWordRegister.CreateRWRegister()},
                {(long)Registers.OwnAddress1, DoubleWordRegister.CreateRWRegister()},
                {(long)Registers.OwnAddress2, DoubleWordRegister.CreateRWRegister()},
                {(long)Registers.NoiseFilter, DoubleWordRegister.CreateRWRegister()},
                {(long)Registers.Control1, control1},
                {(long)Registers.Control2, control2},
                {(long)Registers.Status1, status1},
                {(long)Registers.Status2, status2},
                {(long)Registers.Data, data},
            };
            registers = new DoubleWordRegisterCollection(this, registerDictionary);
        }

        private void InterruptEnableChange(bool oldValue, bool newValue)
        {
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ => Update());
        }

        private void Update()
        {
            EventInterrupt.Set(eventInterruptEnable.Value && (startBit.Value || addressSentOrMatched.Value || byteTransferFinished.Value
                || (bufferInterruptEnable.Value && (dataRegisterEmpty.Value || (dataToReceive?.Any() ?? false)))));
            ErrorInterrupt.Set(errorInterruptEnable.Value && acknowledgeFailed.Value);
        }

        // burst a few RX DMA requests per action; each request makes the
        // DMA read DR, which fetches lazily from the slave. Rescheduled
        // until the transaction ends (STOP from the DMA-complete ISR).
        private void StartRxPump()
        {
            if(pumpActive)
            {
                return;
            }
            pumpActive = true;
            SchedulePump();
        }

        private void SchedulePump()
        {
            machine.ScheduleAction(TimeInterval.FromMicroseconds(PumpIntervalUs), _ =>
            {
                if(state != State.Receiving)
                {
                    pumpActive = false;
                    return;
                }
                for(var i = 0; i < PumpBurst && state == State.Receiving; i++)
                {
                    RxDmaRequest.Blink();
                }
                SchedulePump();
            }, name: "AP_STM32F4_I2C rx pump");
        }

        private uint DataRead()
        {
            // EV8_2 does a dummy DR read purely to clear BTF
            byteTransferFinished.Value = false;

            var result = 0u;
            if(state == State.Receiving)
            {
                if(!(dataToReceive?.Any() ?? false) && selectedSlave != null)
                {
                    var fetched = selectedSlave.Read(1);
                    if(fetched != null && fetched.Length > 0)
                    {
                        (dataToReceive ?? (dataToReceive = new Queue<byte>())).Enqueue(fetched[0]);
                    }
                }
                if(dataToReceive?.Any() ?? false)
                {
                    result = dataToReceive.Dequeue();
                }
            }
            Update();
            return result;
        }

        private void DataWrite(uint newValue)
        {
            byteTransferFinished.Value = false;
            Update();

            switch(state)
            {
            case State.AwaitingAddress:
                startBit.Value = false;
                willReadOnSelectedSlave = (newValue & 1) == 1;
                var address = (int)(newValue >> 1);
                if(ChildCollection.ContainsKey(address))
                {
                    selectedSlave = ChildCollection[address];
                    addressSentOrMatched.Value = true;
                    transmitterReceiver.Value = !willReadOnSelectedSlave;

                    if(willReadOnSelectedSlave)
                    {
                        // EV6_MASTER_REC_MODE_SELECTED is SR1 == ADDR
                        // exactly: no BTF, no RXNE (nothing prefetched -
                        // bytes are fetched lazily by the DMA's DR reads)
                        state = State.Receiving;
                        dataToReceive = new Queue<byte>();
                    }
                    else
                    {
                        state = State.AwaitingData;
                        dataToTransfer = new List<byte>();
                        // EV6_MASTER_TRA_MODE_SELECTED wants ADDR|TXE
                        dataRegisterEmpty.Value = true;
                        if(dmaEnable.Value)
                        {
                            machine.LocalTimeSource.ExecuteInNearestSyncedState(
                                _ => TxDmaRequest.Blink());
                        }
                    }
                }
                else
                {
                    state = State.Idle;
                    busy = false;
                    acknowledgeFailed.Value = true;
                }
                machine.LocalTimeSource.ExecuteInNearestSyncedState(_ => Update());
                break;
            case State.AwaitingData:
                dataToTransfer.Add((byte)newValue);
                // after the (DMA-drained) write the EV8_2 decode wants
                // BTF|TXE with ITEVTEN re-enabled by the DMA-complete ISR
                machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
                {
                    dataRegisterEmpty.Value = true;
                    byteTransferFinished.Value = true;
                    Update();
                });
                break;
            default:
                this.Log(LogLevel.Warning, "Writing 0x{0:X} to DR in unsupported state {1}", newValue, state);
                break;
            }
        }

        private void SoftwareResetWrite(bool oldValue, bool newValue)
        {
            if(newValue)
            {
                Reset();
            }
        }

        private void StopWrite(bool oldValue, bool newValue)
        {
            if(!newValue)
            {
                return;
            }
            if(selectedSlave != null && dataToTransfer != null && dataToTransfer.Count > 0)
            {
                selectedSlave.Write(dataToTransfer.ToArray());
                dataToTransfer.Clear();
            }
            selectedSlave?.FinishTransmission();
            state = State.Idle;
            busy = false;
            byteTransferFinished.Value = false;
            dataRegisterEmpty.Value = false;
            lastDma.Value = false;
            Update();
        }

        private void StartWrite(bool oldValue, bool newValue)
        {
            if(!newValue)
            {
                return;
            }
            // repeated start: flush a pending register write so a
            // write-then-read sees the pointer it just set
            if(selectedSlave != null && dataToTransfer != null && dataToTransfer.Count > 0)
            {
                selectedSlave.Write(dataToTransfer.ToArray());
                dataToTransfer.Clear();
            }
            transmitterReceiver.Value = false;
            dataRegisterEmpty.Value = false;
            byteTransferFinished.Value = false;
            busy = true;
            startBit.Value = true;
            switch(state)
            {
            case State.Idle:
            case State.AwaitingData:
            case State.Receiving:
                state = State.AwaitingAddress;
                masterSlave.Value = true;
                Update();
                break;
            }
        }

        private void PeripheralEnableWrite(bool oldValue, bool newValue)
        {
            if(!newValue)
            {
                acknowledgeEnable.Value = false;
                masterSlave.Value = false;
                acknowledgeFailed.Value = false;
                transmitterReceiver.Value = false;
                dataRegisterEmpty.Value = false;
                byteTransferFinished.Value = false;
                busy = false;
                state = State.Idle;
                Update();
            }
        }

        private IFlagRegisterField acknowledgeEnable;
        private IFlagRegisterField bufferInterruptEnable, eventInterruptEnable, errorInterruptEnable;
        private IFlagRegisterField dmaEnable, lastDma;
        private IFlagRegisterField acknowledgeFailed, dataRegisterEmpty, dataRegisterNotEmpty, byteTransferFinished, addressSentOrMatched, startBit;
        private IFlagRegisterField transmitterReceiver, busyFlag, masterSlave;

        private DoubleWordRegisterCollection registers;

        private State state;
        private bool busy;
        private bool pumpActive;
        private bool addrClearArmed;
        private List<byte> dataToTransfer;
        private Queue<byte> dataToReceive;
        private bool willReadOnSelectedSlave;
        private II2CPeripheral selectedSlave;

        private const int PumpIntervalUs = 100;
        private const int PumpBurst = 8;

        private enum Registers
        {
            Control1 = 0x0,
            Control2 = 0x4,
            OwnAddress1 = 0x8,
            OwnAddress2 = 0xC,
            Data = 0x10,
            Status1 = 0x14,
            Status2 = 0x18,
            ClockControl = 0x1C,
            RiseTime = 0x20,
            NoiseFilter = 0x24,
        }

        private enum State
        {
            Idle,
            AwaitingAddress,
            AwaitingData,
            Receiving,
        }
    }
}
