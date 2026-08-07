//
// Adds the USART IDLE condition used by ChibiOS to harvest a partial
// UART RX DMA buffer. STM32F7_USART models RXNE and DMA requests, but
// its IDLE status and interrupt-enable bits are tags. ArduPilot then
// receives bytes into DMA memory without being told that a MAVLink
// frame ended until the entire bounce buffer fills.
//
// The transition back to an empty USART FIFO happens after DMA reads
// RDR, which is not an idle-line condition: it commonly occurs between
// consecutive bytes. Schedule IDLE after a character-time gap instead.
// A following byte supersedes the pending action. ICR.IDLECF clears the
// synthetic condition.
//
using System;
using System.Reflection;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32F7_USART_Idle : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32F7_USART_Idle(IMachine machine, STM32F7_USART uart)
        {
            this.machine = machine;
            var updateInterrupt = typeof(STM32F7_USART).GetMethod("UpdateInterrupt",
                BindingFlags.NonPublic | BindingFlags.Instance);
            if(updateInterrupt == null)
            {
                throw new ConstructionException("STM32F7_USART no longer has UpdateInterrupt - revisit this fix");
            }

            uart.RegistersCollection.AddAfterReadHook(InterruptAndStatus,
                (long offset, uint value) => idlePending ? value | Idle : value);
            uart.RegistersCollection.AddAfterWriteHook(InterruptFlagClear,
                (long offset, uint value) =>
                {
                    if((value & Idle) == 0)
                    {
                        return;
                    }
                    idlePending = false;
                    updateInterrupt.Invoke(uart, null);
                });

            uart.BufferStateChanged += state =>
            {
                if(state != BufferState.Ready)
                {
                    return;
                }

                var scheduledGeneration = ++generation;
                var baudRate = Math.Max(uart.BaudRate, 1U);
                var delayUs = Math.Max(MinimumIdleDelayUs,
                    (uint)Math.Ceiling(IdleBits * 1000000.0 / baudRate));
                machine.ScheduleAction(TimeInterval.FromMicroseconds(delayUs), _ =>
                {
                    if(scheduledGeneration == generation)
                    {
                        idlePending = true;
                        uart.IRQ.Set(true);
                    }
                }, name: "STM32 USART idle line");
            };
        }

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        public void Reset()
        {
            idlePending = false;
            generation++;
        }

        private readonly IMachine machine;
        private bool idlePending;
        private uint generation;

        private const long InterruptAndStatus = 0x1C;
        private const long InterruptFlagClear = 0x20;
        private const uint Idle = 1 << 4;
        private const uint IdleBits = 11;
        // AP_UARTPacer uses a 500us minimum inter-byte interval. Keep the
        // synthetic idle gap well clear of queued external byte events.
        private const uint MinimumIdleDelayUs = 2000;
    }
}
