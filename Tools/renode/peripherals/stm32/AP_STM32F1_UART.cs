// STM32F1 UART wrapper with an IDLE interval suitable for a paced serial link.
// The stock STM32_UART schedules IDLE eight bit-times after each received byte,
// which is earlier than the following 8N1 byte. ChibiOS uses IDLE to terminate
// compact IOMCU packets, so that behavior splits a request at every byte.
using System;
using System.Reflection;
using System.Threading;

using Antmicro.Migrant;
using Antmicro.Migrant.Hooks;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    [AllowedTranslations(AllowedTranslation.WordToDoubleWord |
        AllowedTranslation.ByteToDoubleWord)]
    public class AP_STM32F1_UART : IDoubleWordPeripheral, IUART, IKnownSize,
        IHasFrequency
    {
        public AP_STM32F1_UART(IMachine machine, uint frequency = 8000000)
        {
            this.machine = machine;
            uart = new STM32_UART(machine, frequency);
            uart.CharReceived += value => CharReceived?.Invoke(value);
            ResolveReflection();
        }

        private void ResolveReflection()
        {
            var uartType = typeof(STM32_UART);
            idleCancellation = uartType.GetField("idleLineDetectedCancellationTokenSrc",
                BindingFlags.NonPublic | BindingFlags.Instance);
            idleLineDetected = uartType.GetField("idleLineDetected",
                BindingFlags.NonPublic | BindingFlags.Instance);
            uartFrequency = uartType.GetField("frequency",
                BindingFlags.NonPublic | BindingFlags.Instance);
            updateInterrupt = uartType.GetMethod("Update",
                BindingFlags.NonPublic | BindingFlags.Instance);
            if(idleCancellation == null || idleLineDetected == null ||
                uartFrequency == null || updateInterrupt == null)
            {
                throw new ConstructionException(
                    "STM32_UART IDLE internals changed - revisit the STM32F1 wrapper");
            }
        }

        [PostDeserialization]
        private void AfterDeserialization()
        {
            ResolveReflection();
        }

        public void Reset()
        {
            generation++;
            uart.Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            return uart.ReadDoubleWord(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            // These interrupt controls have no corresponding conditions in
            // the stock byte-level UART. TX DMA writes the data register
            // through the DMA model without a UART request output. Masking
            // the tag-only bits avoids a warning on every IOMCU transaction.
            var innerValue = value;
            switch(offset)
            {
            case Control1:
                innerValue &= ~(ParityErrorInterruptEnable | WordLength);
                break;
            case Control2:
                innerValue &= ~LineBreakInterruptEnable;
                break;
            case Control3:
                innerValue &= ~(ErrorInterruptEnable | TransmitDmaEnable);
                break;
            }
            uart.WriteDoubleWord(offset, innerValue);
            if(offset == Control1 &&
                (value & (ReceiverEnable | UartEnable)) !=
                (ReceiverEnable | UartEnable))
            {
                generation++;
            }
        }

        public void WriteChar(byte value)
        {
            uart.WriteChar(value);
            ((CancellationTokenSource)idleCancellation.GetValue(uart))?.Cancel();

            var scheduledGeneration = ++generation;
            var baudRate = Math.Max(BaudRate, 1U);
            var delayUs = (uint)Math.Ceiling(IdleBits * 1000000.0 / baudRate);
            machine.ScheduleAction(TimeInterval.FromMicroseconds(delayUs), _ =>
            {
                if(scheduledGeneration != generation)
                {
                    return;
                }
                ((IFlagRegisterField)idleLineDetected.GetValue(uart)).Value = true;
                updateInterrupt.Invoke(uart, null);
            }, name: "STM32F1 USART idle line");
        }

        public uint BaudRate => uart.BaudRate;
        public Bits StopBits => uart.StopBits;
        public Parity ParityBit => uart.ParityBit;
        public long Size => 0x100;

        public ulong Frequency
        {
            get => (uint)uartFrequency.GetValue(uart);
            set => uartFrequency.SetValue(uart, checked((uint)value));
        }

        [DefaultInterrupt]
        public GPIO IRQ => uart.IRQ;

        public GPIO DMARequest => uart.DMARequest;

        [field: Transient]
        public event Action<byte> CharReceived;

        private readonly IMachine machine;
        private readonly STM32_UART uart;
        private uint generation;

        [Transient]
        private FieldInfo idleCancellation;
        [Transient]
        private FieldInfo idleLineDetected;
        [Transient]
        private FieldInfo uartFrequency;
        [Transient]
        private MethodInfo updateInterrupt;

        private const long Control1 = 0x0C;
        private const long Control2 = 0x10;
        private const long Control3 = 0x14;
        private const uint ReceiverEnable = 1U << 2;
        private const uint ParityErrorInterruptEnable = 1U << 8;
        private const uint WordLength = 1U << 12;
        private const uint UartEnable = 1U << 13;
        private const uint LineBreakInterruptEnable = 1U << 6;
        private const uint ErrorInterruptEnable = 1U << 0;
        private const uint TransmitDmaEnable = 1U << 7;
        // Wait beyond the next 8N1 byte boundary. A following byte cancels the
        // pending action before a compact packet is reported as idle.
        private const uint IdleBits = 20;
    }
}
