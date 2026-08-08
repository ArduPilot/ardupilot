// Persist a region of emulated address space when the machine pauses or is
// disposed. This backs the complete internal STM32 flash: Renode's STM32 flash
// controllers modify MappedMemory, which has no backing file of its own.
//
using System;
using System.IO;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_PersistentMemory : IPeripheral, IDisposable
    {
        public AP_PersistentMemory(IMachine machine, string fileName, ulong address, int size)
        {
            this.machine = machine;
            this.fileName = fileName;
            this.address = address;
            this.size = size;
            machine.StateChanged += OnMachineStateChanged;
        }

        public void Reset()
        {
        }

        public void Dispose()
        {
            machine.StateChanged -= OnMachineStateChanged;
            Save();
        }

        private void OnMachineStateChanged(IMachine sender, MachineStateChangedEventArgs args)
        {
            if(args.CurrentState == MachineStateChangedEventArgs.State.Paused)
            {
                Save();
            }
        }

        private void Save()
        {
            try
            {
                var directory = Path.GetDirectoryName(fileName);
                if(!string.IsNullOrEmpty(directory))
                {
                    Directory.CreateDirectory(directory);
                }
                File.WriteAllBytes(fileName, machine.SystemBus.ReadBytes(address, size));
            }
            catch(Exception error)
            {
                this.Log(LogLevel.Error, "Failed to persist {0}: {1}", fileName, error.Message);
            }
        }

        private readonly IMachine machine;
        private readonly string fileName;
        private readonly ulong address;
        private readonly int size;
    }
}
