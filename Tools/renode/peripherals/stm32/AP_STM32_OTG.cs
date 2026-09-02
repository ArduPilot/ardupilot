// STM32 Synopsys OTG device controller used by the ChibiOS OTGv1 driver.
//
// USB transactions come from Renode's USB/IP server through USBDeviceCore.
// Setup packets and endpoint data are then presented through the STM32 RX/TX
// FIFOs and interrupts, so descriptors and class behaviour remain entirely
// under the control of the emulated firmware.
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.USB;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.USB
{
    public class AP_STM32_OTG : IDoubleWordPeripheral, IKnownSize, IUSBDevice
    {
        public AP_STM32_OTG(IMachine machine,
                            bool connectOnVbusSensing = false)
        {
            this.machine = machine;
            this.connectOnVbusSensing = connectOnVbusSensing;
            sofTimer = new LimitTimer(machine.ClockSource, 1000, this,
                "USB start of frame", limit: 1,
                direction: Antmicro.Renode.Time.Direction.Ascending, enabled: false,
                workMode: Antmicro.Renode.Time.WorkMode.Periodic,
                eventEnabled: true);
            sofTimer.LimitReached += () =>
            {
                lock(sync)
                {
                    HandleStartOfFrame();
                }
            };
            IRQ = new GPIO();
            inEndpoints = new EndpointState[EndpointCount];
            outEndpoints = new EndpointState[EndpointCount];
            usbInEndpoints = new USBEndpoint[EndpointCount];
            for(var endpoint = 0; endpoint < EndpointCount; endpoint++)
            {
                inEndpoints[endpoint] = new EndpointState();
                outEndpoints[endpoint] = new EndpointState();
            }

            USBCore = new USBDeviceCore(this, customSetupPacketHandler: HandleSetupPacket);
            USBCore.WithConfiguration(configure: configuration =>
                configuration.WithInterface(configure: iface =>
                {
                    for(byte endpoint = 1; endpoint < EndpointCount; endpoint++)
                    {
                        USBEndpoint usbIn;
                        USBEndpoint usbOut;
                        // Firmware supplies the real descriptors, but this
                        // value controls USB/IP payload chunking.
                        iface.WithEndpoint(Direction.DeviceToHost,
                            EndpointTransferType.Bulk, InternalPacketSize, 1,
                            out usbIn, endpoint);
                        iface.WithEndpoint(Direction.HostToDevice,
                            EndpointTransferType.Bulk, InternalPacketSize, 1,
                            out usbOut, endpoint);
                        usbIn.NonBlocking = true;
                        usbInEndpoints[endpoint] = usbIn;
                        var capturedEndpoint = endpoint;
                        usbOut.DataWritten += data =>
                            QueueHostData(capturedEndpoint, data);
                    }
                }));
            genericConfiguration =
                new List<USBConfiguration>(USBCore.Configurations)[0];
            Reset();
        }

        public void Reset()
        {
            lock(sync)
            {
                if(connected)
                {
                    SetUSBIPConnected(false);
                }
                registers.Clear();
                rxQueue.Clear();
                pendingOut.Clear();
                Array.Clear(activeOut, 0, activeOut.Length);
                activeRx = null;
                setupResponse = null;
                setupResponseData.Clear();
                setupAdditionalData = Array.Empty<byte>();
                setupAdditionalOffset = 0;
                setupConfigurationDescriptor = false;
                setupExpectedLength = 0;
                connected = false;
                enumerateAfterReset = false;
                fifoReadyMask = 0;
                frameNumber = 0;
                sofTimer.Enabled = false;
                foreach(var endpoint in inEndpoints)
                {
                    endpoint.Reset();
                }
                foreach(var endpoint in outEndpoints)
                {
                    endpoint.Reset();
                }
                registers[DeviceControl] = SoftDisconnect;
                USBCore.Reset();
                for(var endpoint = 1; endpoint < EndpointCount; endpoint++)
                {
                    usbInEndpoints[endpoint].NonBlocking = true;
                    SetAutoTerminateTransfers(usbInEndpoints[endpoint], true);
                }
                USBCore.SelectedConfiguration = genericConfiguration;
                IRQ.Unset();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            lock(sync)
            {
                if(offset >= FifoBase)
                {
                    return ReadFifo(offset);
                }
                if(IsInEndpointRegister(offset, out var endpoint,
                    out var register))
                {
                    return ReadEndpoint(inEndpoints[endpoint], register,
                        true, endpoint);
                }
                if(IsOutEndpointRegister(offset, out endpoint, out register))
                {
                    return ReadEndpoint(outEndpoints[endpoint], register,
                        false, endpoint);
                }
                switch(offset)
                {
                case ResetControl:
                    return AhbIdle;
                case GlobalInterruptStatus:
                    return GetGlobalInterruptStatus();
                case ReceiveStatusDebug:
                    return rxQueue.Count == 0 ? 0 : rxQueue.Peek().Status;
                case ReceiveStatusPop:
                    return PopReceiveStatus();
                case DeviceStatus:
                    return FullSpeed48MHz |
                        ((uint)frameNumber << FrameNumberShift);
                case AllEndpointsInterrupt:
                    return GetAllEndpointsInterrupt();
                default:
                    return GetRegister(offset);
                }
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            lock(sync)
            {
                if(offset >= FifoBase)
                {
                    WriteFifo(offset, value);
                    return;
                }
                if(IsInEndpointRegister(offset, out var endpoint,
                    out var register))
                {
                    WriteEndpoint(inEndpoints[endpoint], register, value,
                        true, endpoint);
                    return;
                }
                if(IsOutEndpointRegister(offset, out endpoint, out register))
                {
                    WriteEndpoint(outEndpoints[endpoint], register, value,
                        false, endpoint);
                    return;
                }
                switch(offset)
                {
                case AhbConfiguration:
                    WriteAhbConfiguration(value);
                    break;
                case ResetControl:
                    // Core reset and FIFO flush request bits self-clear.
                    break;
                case GlobalInterruptStatus:
                    var hadReset =
                        (GetRegister(offset) & UsbResetInterrupt) != 0;
                    registers[offset] = GetRegister(offset) & ~value;
                    if(hadReset && (value & UsbResetInterrupt) != 0 &&
                        enumerateAfterReset)
                    {
                        enumerateAfterReset = false;
                        registers[offset] |= EnumerationDoneInterrupt;
                    }
                    break;
                case GlobalCoreConfiguration:
                    WriteGlobalCoreConfiguration(value);
                    break;
                case DeviceControl:
                    WriteDeviceControl(value);
                    break;
                case DeviceConfiguration:
                    registers[offset] = value;
                    USBCore.Address = (byte)((value & DeviceAddressMask) >>
                        DeviceAddressShift);
                    break;
                case InEndpointFifoEmptyMask:
                    WriteFifoEmptyMask(value);
                    break;
                default:
                    registers[offset] = value;
                    break;
                }
                UpdateInterrupts();
            }
        }

        public USBDeviceCore USBCore { get; }

        public GPIO IRQ { get; }

        public long Size => 0x40000;

        private void HandleSetupPacket(SetupPacket packet,
            byte[] additionalData, Action<byte[]> response)
        {
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
            {
                lock(sync)
                {
                    if(!connected)
                    {
                        this.Log(LogLevel.Warning,
                            "USB setup packet received while disconnected");
                    }
                    inEndpoints[0].Control &= ~EndpointStall;
                    outEndpoints[0].Control &= ~EndpointStall;
                    if(setupResponse != null)
                    {
                        this.Log(LogLevel.Warning,
                            "Replacing an unfinished USB setup transaction");
                        setupResponse(Array.Empty<byte>());
                    }
                    setupResponse = response;
                    setupConfigurationDescriptor =
                        packet.Type == PacketType.Standard &&
                        packet.Request == (byte)StandardRequest.GetDescriptor &&
                        (packet.Value >> 8) == ConfigurationDescriptorType;
                    setupExpectedLength = packet.Count;
                    setupResponseData.Clear();
                    setupAdditionalData =
                        additionalData ?? Array.Empty<byte>();
                    setupAdditionalOffset = 0;

                    var encoded = new byte[]
                    {
                        (byte)((byte)packet.Recipient |
                            ((byte)packet.Type << 5) |
                            ((byte)packet.Direction << 7)),
                        packet.Request,
                        (byte)packet.Value,
                        (byte)(packet.Value >> 8),
                        (byte)packet.Index,
                        (byte)(packet.Index >> 8),
                        (byte)packet.Count,
                        (byte)(packet.Count >> 8),
                    };
                    rxQueue.Enqueue(new ReceivePacket(
                        MakeReceiveStatus(0, encoded.Length, SetupDataStatus),
                        encoded,
                        () => SetOutInterrupt(0, SetupInterrupt)));
                    UpdateInterrupts();
                }
            });
        }

        private void QueueHostData(byte endpoint, byte[] data)
        {
            var copy = (byte[])data.Clone();
            this.Log(LogLevel.Debug, "USB OUT endpoint {0}: {1}",
                endpoint, BitConverter.ToString(copy));
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
            {
                lock(sync)
                {
                    this.Log(LogLevel.Debug,
                        "Delivering USB OUT endpoint {0}: enabled={1}, size={2}, transferred={3}",
                        endpoint, outEndpoints[endpoint].Enabled,
                        outEndpoints[endpoint].TransferSize, outEndpoints[endpoint].Transferred);
                    pendingOut.Enqueue(new HostPacket(endpoint, copy));
                    TryDeliverOut(endpoint);
                    UpdateInterrupts();
                }
            });
        }

        private uint ReadFifo(long offset)
        {
            if((offset - FifoBase) / FifoStride != 0 || activeRx == null)
            {
                return 0;
            }
            uint value = 0;
            for(var i = 0; i < 4; i++)
            {
                if(activeRx.Offset < activeRx.Data.Length)
                {
                    value |= (uint)activeRx.Data[activeRx.Offset++] << (i * 8);
                }
            }
            if(activeRx.Offset >= activeRx.Data.Length)
            {
                FinishActiveReceive();
            }
            return value;
        }

        private void WriteFifo(long offset, uint value)
        {
            var endpoint = (int)((offset - FifoBase) / FifoStride);
            if(endpoint < 0 || endpoint >= EndpointCount)
            {
                return;
            }
            var state = inEndpoints[endpoint];
            var remaining = state.TransferSize - state.Transferred;
            for(var i = 0; i < 4 && remaining > 0; i++, remaining--)
            {
                state.Data.Add((byte)(value >> (i * 8)));
                state.Transferred++;
            }
            if(state.Enabled && state.Transferred >= state.TransferSize)
            {
                CompleteInTransfer(endpoint);
            }
            UpdateInterrupts();
        }

        private uint PopReceiveStatus()
        {
            if(activeRx != null)
            {
                this.Log(LogLevel.Warning,
                    "USB receive status popped before payload was read");
                FinishActiveReceive();
            }
            if(rxQueue.Count == 0)
            {
                return 0;
            }
            activeRx = rxQueue.Dequeue();
            var status = activeRx.Status;
            if(activeRx.Data.Length == 0)
            {
                FinishActiveReceive();
            }
            UpdateInterrupts();
            return status;
        }

        private void FinishActiveReceive()
        {
            var completed = activeRx;
            this.Log(LogLevel.Debug, "Firmware consumed RX status 0x{0:X8} ({1} bytes)",
                completed == null ? 0 : completed.Status,
                completed == null ? 0 : completed.Data.Length);
            activeRx = null;
            UpdateInterrupts();
            if(completed?.Completed == null)
            {
                return;
            }
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
            {
                lock(sync)
                {
                    completed.Completed();
                    UpdateInterrupts();
                }
            });
        }

        private void WriteFifoEmptyMask(uint value)
        {
            registers[InEndpointFifoEmptyMask] = value;
            fifoReadyMask &= value;
            var pending = value & ~fifoReadyMask;
            if(pending == 0)
            {
                return;
            }
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
            {
                lock(sync)
                {
                    fifoReadyMask |= pending &
                        GetRegister(InEndpointFifoEmptyMask);
                    UpdateInterrupts();
                }
            });
        }

        private uint ReadEndpoint(EndpointState state, long register,
            bool input, int endpoint)
        {
            switch(register)
            {
            case EndpointControl:
                return state.Control;
            case EndpointInterrupt:
                var value = state.Interrupt;
                if(input && EndpointNeedsFifo(endpoint))
                {
                    value |= TransmitFifoEmptyInterrupt;
                }
                return value;
            case EndpointTransferSize:
                return state.TransferSizeRegister;
            case EndpointTransmitFifoStatus:
                return input ? TransmitFifoWordsAvailable : 0;
            default:
                return state.GetRegister(register);
            }
        }

        private void WriteEndpoint(EndpointState state, long register,
            uint value, bool input, int endpoint)
        {
            switch(register)
            {
            case EndpointControl:
                state.Control = value;
                if(endpoint == 0 && input &&
                    (value & EndpointStall) != 0 && setupResponse != null)
                {
                    var response = setupResponse;
                    setupResponse = null;
                    setupResponseData.Clear();
                    response(Array.Empty<byte>());
                }
                if((value & EndpointDisable) != 0)
                {
                    state.Enabled = false;
                    state.Control &= ~EndpointEnable;
                    state.Interrupt |= EndpointDisabledInterrupt;
                }
                else if((value & EndpointEnable) != 0)
                {
                    state.Enabled = true;
                    state.Control |= EndpointEnable;
                    if(input)
                    {
                        if(state.TransferSize == 0)
                        {
                            CompleteInTransfer(endpoint);
                        }
                    }
                    else
                    {
                        state.Transferred = 0;
                        TryDeliverOut(endpoint);
                    }
                }
                break;
            case EndpointInterrupt:
                state.Interrupt &= ~value;
                break;
            case EndpointTransferSize:
                state.TransferSizeRegister = value;
                state.TransferSize = (int)(value & TransferSizeMask);
                state.Transferred = 0;
                state.Data.Clear();
                break;
            default:
                state.SetRegister(register, value);
                break;
            }
            UpdateInterrupts();
        }

        private void CompleteInTransfer(int endpoint)
        {
            var state = inEndpoints[endpoint];
            if(!state.Enabled)
            {
                return;
            }
            state.Enabled = false;
            state.Control &= ~EndpointEnable;
            var data = state.Data.ToArray();
            this.Log(LogLevel.Debug, "USB IN endpoint {0}: {1}",
                endpoint, BitConverter.ToString(data));
            state.Data.Clear();

            if(endpoint == 0)
            {
                setupResponseData.AddRange(data);
                if(setupResponse != null &&
                    (data.Length < EndpointZeroPacketSize ||
                     setupResponseData.Count >= setupExpectedLength))
                {
                    var response = setupResponse;
                    var responseData = setupResponseData.ToArray();
                    if(setupConfigurationDescriptor)
                    {
                        ConfigureInEndpointReads(responseData);
                    }
                    setupResponse = null;
                    setupConfigurationDescriptor = false;
                    setupResponseData.Clear();
                    response(responseData);
                }
            }
            else if(data.Length != 0 ||
                !GetAutoTerminateTransfers(usbInEndpoints[endpoint]))
            {
                usbInEndpoints[endpoint].HandlePacket(data);
            }
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
            {
                lock(sync)
                {
                    state.Interrupt |= TransferCompleteInterrupt;
                    UpdateInterrupts();
                }
            });
        }

        private void ConfigureInEndpointReads(byte[] descriptor)
        {
            for(var endpoint = 1; endpoint < EndpointCount; endpoint++)
            {
                usbInEndpoints[endpoint].NonBlocking = true;
                SetAutoTerminateTransfers(usbInEndpoints[endpoint], true);
            }

            var massStorageInterface = false;
            var offset = 0;
            while(offset + 1 < descriptor.Length)
            {
                var length = descriptor[offset];
                if(length < 2 || offset + length > descriptor.Length)
                {
                    break;
                }
                var type = descriptor[offset + 1];
                if(type == InterfaceDescriptorType && length >= 9)
                {
                    massStorageInterface =
                        descriptor[offset + 5] == MassStorageClass;
                }
                else if(type == EndpointDescriptorType && length >= 7 &&
                    massStorageInterface)
                {
                    var address = descriptor[offset + 2];
                    var endpoint = address & EndpointAddressMask;
                    if((address & EndpointDirectionIn) != 0 &&
                        endpoint > 0 && endpoint < EndpointCount)
                    {
                        usbInEndpoints[endpoint].NonBlocking = false;
                        if(!SetAutoTerminateTransfers(
                            usbInEndpoints[endpoint], false))
                        {
                            this.Log(LogLevel.Warning,
                                "USB mass storage requires the USB/IP Renode patch");
                        }
                    }
                }
                offset += length;
            }
        }

        private void TryDeliverOut(int endpoint)
        {
            if(endpoint == 0 &&
                setupAdditionalOffset < setupAdditionalData.Length)
            {
                DeliverOutData(0, setupAdditionalData,
                    ref setupAdditionalOffset);
                return;
            }
            if(endpoint == 0 || !outEndpoints[endpoint].Enabled)
            {
                return;
            }

            var packet = activeOut[endpoint];
            if(packet == null)
            {
                // Avoid head-of-line blocking between the two CDC functions.
                var count = pendingOut.Count;
                while(count-- > 0)
                {
                    var candidate = pendingOut.Dequeue();
                    if(packet == null && candidate.Endpoint == endpoint)
                    {
                        packet = candidate;
                    }
                    else
                    {
                        pendingOut.Enqueue(candidate);
                    }
                }
                if(packet == null)
                {
                    return;
                }
                activeOut[endpoint] = packet;
            }
            var offset = packet.Offset;
            DeliverOutData(endpoint, packet.Data, ref offset);
            packet.Offset = offset;
            if(packet.Offset >= packet.Data.Length)
            {
                activeOut[endpoint] = null;
            }
        }

        private void DeliverOutData(int endpoint, byte[] data, ref int offset)
        {
            var state = outEndpoints[endpoint];
            if(!state.Enabled)
            {
                return;
            }
            var maximumPacket = GetMaximumPacketSize(state.Control);
            var remaining =
                Math.Max(0, state.TransferSize - state.Transferred);
            var length = Math.Min(maximumPacket,
                Math.Min(data.Length - offset, remaining));
            var packet = new byte[length];
            Array.Copy(data, offset, packet, 0, length);
            offset += length;
            state.Transferred += length;
            var complete = length < maximumPacket ||
                state.Transferred >= state.TransferSize;
            if(complete)
            {
                // Do not accept another host packet until firmware has
                // handled XFRC and re-enabled the endpoint. USB/IP can queue
                // multiple URBs before the guest consumes this FIFO entry.
                state.Enabled = false;
                state.Control &= ~EndpointEnable;
            }
            rxQueue.Enqueue(new ReceivePacket(
                MakeReceiveStatus(endpoint, length, OutDataStatus), packet,
                () =>
                {
                    if(complete)
                    {
                        SetOutInterrupt(endpoint, TransferCompleteInterrupt);
                    }
                    else
                    {
                        TryDeliverOut(endpoint);
                    }
                }));
        }

        private void WriteDeviceControl(uint value)
        {
            var wasDisconnected =
                (GetRegister(DeviceControl) & SoftDisconnect) != 0;
            registers[DeviceControl] = value;
            if((value & SoftDisconnect) != 0)
            {
                Disconnect();
                return;
            }
            if(wasDisconnected && !connectOnVbusSensing)
            {
                Connect();
            }
        }

        private void WriteGlobalCoreConfiguration(uint value)
        {
            if(!connectOnVbusSensing)
            {
                registers[GlobalCoreConfiguration] = value;
                return;
            }
            var wasConnected =
                (GetRegister(GlobalCoreConfiguration) & VbusBSensingEnable) != 0;
            registers[GlobalCoreConfiguration] = value;
            var isConnected = (value & VbusBSensingEnable) != 0;
            if(wasConnected == isConnected)
            {
                return;
            }
            if(!isConnected)
            {
                Disconnect();
                return;
            }
            // STM32F4 usbStart() enables VBUS sensing before clearing pending
            // interrupts. Connect only once it has completed initialization
            // by enabling the controller's global interrupt below.
            if((GetRegister(AhbConfiguration) & GlobalInterruptEnable) != 0)
            {
                Connect();
            }
        }

        private void WriteAhbConfiguration(uint value)
        {
            var wasEnabled =
                (GetRegister(AhbConfiguration) & GlobalInterruptEnable) != 0;
            registers[AhbConfiguration] = value;
            if(!connectOnVbusSensing)
            {
                return;
            }
            if((value & GlobalInterruptEnable) == 0)
            {
                if(wasEnabled)
                {
                    Disconnect();
                }
                return;
            }
            if((GetRegister(GlobalCoreConfiguration) &
                VbusBSensingEnable) != 0)
            {
                Connect();
            }
        }

        private void Connect()
        {
            if(connected)
            {
                return;
            }
            connected = SetUSBIPConnected(true);
            if(!connected)
            {
                return;
            }
            sofTimer.Enabled = true;
            USBCore.Address = 0;
            USBCore.SelectedConfiguration = genericConfiguration;
            registers[GlobalInterruptStatus] =
                GetRegister(GlobalInterruptStatus) | UsbResetInterrupt;
            enumerateAfterReset = true;
        }

        private void Disconnect()
        {
            if(connected)
            {
                SetUSBIPConnected(false);
            }
            connected = false;
            sofTimer.Enabled = false;
        }

        private bool SetUSBIPConnected(bool deviceConnected)
        {
            var host = EmulationManager.Instance.CurrentEmulation.HostMachine;
            var server = host.TryGetByName("usb", out var found);
            if(!found)
            {
                return false;
            }
            var setConnected =
                server.GetType().GetMethod("SetDeviceConnected");
            if(setConnected == null)
            {
                this.Log(LogLevel.Warning,
                    "USB/IP server does not support device connection state");
                return false;
            }
            try
            {
                setConnected.Invoke(server, new object[] { deviceConnected });
                return true;
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Warning,
                    "Failed to update USB/IP device state: {0}", e.Message);
                return false;
            }
        }

        private static bool GetAutoTerminateTransfers(USBEndpoint endpoint)
        {
            return AutoTerminateTransfersProperty == null ||
                (bool)AutoTerminateTransfersProperty.GetValue(endpoint);
        }

        private static bool SetAutoTerminateTransfers(USBEndpoint endpoint,
            bool value)
        {
            if(AutoTerminateTransfersProperty == null)
            {
                return false;
            }
            AutoTerminateTransfersProperty.SetValue(endpoint, value);
            return true;
        }

        private void HandleStartOfFrame()
        {
            frameNumber = (frameNumber + 1) & FrameNumberMask;
            if(!connected)
            {
                return;
            }
            registers[GlobalInterruptStatus] =
                GetRegister(GlobalInterruptStatus) | StartOfFrameInterrupt;
            UpdateInterrupts();
        }

        private uint GetGlobalInterruptStatus()
        {
            var status = GetRegister(GlobalInterruptStatus);
            if(rxQueue.Count != 0)
            {
                status |= ReceiveFifoLevelInterrupt;
            }
            if(HasEndpointInterrupt(true))
            {
                status |= InEndpointInterrupt;
            }
            if(HasEndpointInterrupt(false))
            {
                status |= OutEndpointInterrupt;
            }
            return status;
        }

        private uint GetAllEndpointsInterrupt()
        {
            uint value = 0;
            for(var endpoint = 0; endpoint < EndpointCount; endpoint++)
            {
                if(GetMaskedEndpointInterrupt(endpoint, true) != 0)
                {
                    value |= 1u << endpoint;
                }
                if(GetMaskedEndpointInterrupt(endpoint, false) != 0)
                {
                    value |= 1u << (endpoint + 16);
                }
            }
            return value;
        }

        private bool HasEndpointInterrupt(bool input)
        {
            var allMask = GetRegister(AllEndpointsInterruptMask);
            for(var endpoint = 0; endpoint < EndpointCount; endpoint++)
            {
                var bit = input ? endpoint : endpoint + 16;
                if((allMask & (1u << bit)) != 0 &&
                    GetMaskedEndpointInterrupt(endpoint, input) != 0)
                {
                    return true;
                }
            }
            return false;
        }

        private uint GetMaskedEndpointInterrupt(int endpoint, bool input)
        {
            var state = input ? inEndpoints[endpoint] :
                outEndpoints[endpoint];
            var mask = GetRegister(input ? InEndpointInterruptMask :
                OutEndpointInterruptMask);
            var interrupt = state.Interrupt & mask;
            if(input && EndpointNeedsFifo(endpoint))
            {
                interrupt |= TransmitFifoEmptyInterrupt;
            }
            return interrupt;
        }

        private bool EndpointNeedsFifo(int endpoint)
        {
            return inEndpoints[endpoint].Enabled &&
                inEndpoints[endpoint].Transferred <
                    inEndpoints[endpoint].TransferSize &&
                (fifoReadyMask &
                    (1u << endpoint)) != 0;
        }

        private void SetOutInterrupt(int endpoint, uint interrupt)
        {
            this.Log(LogLevel.Debug, "USB OUT endpoint {0} interrupt 0x{1:X}",
                endpoint, interrupt);
            outEndpoints[endpoint].Interrupt |= interrupt;
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            var enabled =
                (GetRegister(AhbConfiguration) & GlobalInterruptEnable) != 0;
            IRQ.Set(enabled &&
                (GetGlobalInterruptStatus() &
                    GetRegister(GlobalInterruptMask)) != 0);
        }

        private uint GetRegister(long offset)
        {
            return registers.TryGetValue(offset, out var value) ? value : 0;
        }

        private static bool IsInEndpointRegister(long offset,
            out int endpoint, out long register)
        {
            endpoint = (int)((offset - InEndpointBase) / EndpointStride);
            register = (offset - InEndpointBase) % EndpointStride;
            return offset >= InEndpointBase && offset < OutEndpointBase &&
                endpoint >= 0 && endpoint < EndpointCount;
        }

        private static bool IsOutEndpointRegister(long offset,
            out int endpoint, out long register)
        {
            endpoint = (int)((offset - OutEndpointBase) / EndpointStride);
            register = (offset - OutEndpointBase) % EndpointStride;
            return offset >= OutEndpointBase && offset < OutEndpointLimit &&
                endpoint >= 0 && endpoint < EndpointCount;
        }

        private static uint MakeReceiveStatus(int endpoint, int length,
            uint status)
        {
            return (uint)endpoint | ((uint)length << 4) | (status << 17);
        }

        private static int GetMaximumPacketSize(uint control)
        {
            var size = (int)(control & MaximumPacketSizeMask);
            return size == 0 ? EndpointZeroPacketSize : size;
        }

        private readonly IMachine machine;
        private readonly bool connectOnVbusSensing;
        private readonly LimitTimer sofTimer;
        private readonly object sync = new object();
        private readonly Dictionary<long, uint> registers =
            new Dictionary<long, uint>();
        private readonly EndpointState[] inEndpoints;
        private readonly EndpointState[] outEndpoints;
        private readonly USBEndpoint[] usbInEndpoints;
        private readonly Queue<ReceivePacket> rxQueue =
            new Queue<ReceivePacket>();
        private readonly Queue<HostPacket> pendingOut =
            new Queue<HostPacket>();
        private readonly HostPacket[] activeOut =
            new HostPacket[EndpointCount];
        private readonly USBConfiguration genericConfiguration;

        private static readonly System.Reflection.PropertyInfo
            AutoTerminateTransfersProperty = typeof(USBEndpoint).GetProperty(
                "AutoTerminateTransfers");

        private ReceivePacket activeRx;
        private Action<byte[]> setupResponse;
        private readonly List<byte> setupResponseData = new List<byte>();
        private byte[] setupAdditionalData = Array.Empty<byte>();
        private int setupAdditionalOffset;
        private bool setupConfigurationDescriptor;
        private int setupExpectedLength;
        private int frameNumber;
        private bool connected;
        private bool enumerateAfterReset;
        private uint fifoReadyMask;

        private const int EndpointCount = 9;
        private const short InternalPacketSize = 64;
        private const int EndpointZeroPacketSize = 64;
        private const int ConfigurationDescriptorType = 2;
        private const int InterfaceDescriptorType = 4;
        private const int EndpointDescriptorType = 5;
        private const int MassStorageClass = 8;
        private const int EndpointAddressMask = 0x0F;
        private const int EndpointDirectionIn = 0x80;
        private const long AhbConfiguration = 0x008;
        private const long GlobalCoreConfiguration = 0x038;
        private const long ResetControl = 0x010;
        private const long GlobalInterruptStatus = 0x014;
        private const long GlobalInterruptMask = 0x018;
        private const long ReceiveStatusDebug = 0x01C;
        private const long ReceiveStatusPop = 0x020;
        private const long DeviceConfiguration = 0x800;
        private const long DeviceControl = 0x804;
        private const long DeviceStatus = 0x808;
        private const long InEndpointInterruptMask = 0x810;
        private const long OutEndpointInterruptMask = 0x814;
        private const long AllEndpointsInterrupt = 0x818;
        private const long AllEndpointsInterruptMask = 0x81C;
        private const long InEndpointFifoEmptyMask = 0x834;
        private const long InEndpointBase = 0x900;
        private const long OutEndpointBase = 0xB00;
        private const long OutEndpointLimit = 0xD00;
        private const long EndpointStride = 0x20;
        private const long EndpointControl = 0x00;
        private const long EndpointInterrupt = 0x08;
        private const long EndpointTransferSize = 0x10;
        private const long EndpointTransmitFifoStatus = 0x18;
        private const long FifoBase = 0x1000;
        private const long FifoStride = 0x1000;
        private const uint AhbIdle = 1u << 31;
        private const uint GlobalInterruptEnable = 1u << 0;
        private const uint VbusBSensingEnable = 1u << 19;
        private const uint OutEndpointInterrupt = 1u << 19;
        private const uint InEndpointInterrupt = 1u << 18;
        private const uint EnumerationDoneInterrupt = 1u << 13;
        private const uint UsbResetInterrupt = 1u << 12;
        private const uint ReceiveFifoLevelInterrupt = 1u << 4;
        private const uint StartOfFrameInterrupt = 1u << 3;
        private const uint FullSpeed48MHz = 3u << 1;
        private const uint SoftDisconnect = 1u << 1;
        private const uint DeviceAddressMask = 0x7Fu << 4;
        private const int DeviceAddressShift = 4;
        private const uint EndpointEnable = 1u << 31;
        private const uint EndpointDisable = 1u << 30;
        private const uint EndpointStall = 1u << 21;
        private const uint TransmitFifoEmptyInterrupt = 1u << 7;
        private const uint SetupInterrupt = 1u << 3;
        private const uint EndpointDisabledInterrupt = 1u << 1;
        private const uint TransferCompleteInterrupt = 1u << 0;
        private const uint TransferSizeMask = 0x7FFFFu;
        private const uint MaximumPacketSizeMask = 0x3FFu;
        private const uint TransmitFifoWordsAvailable = 0xFFFFu;
        private const uint OutDataStatus = 2;
        private const uint SetupDataStatus = 6;
        private const int FrameNumberShift = 8;
        private const int FrameNumberMask = 0x3FF;

        private class EndpointState
        {
            public void Reset()
            {
                Registers.Clear();
                Data.Clear();
                Control = 0;
                Interrupt = 0;
                TransferSizeRegister = 0;
                TransferSize = 0;
                Transferred = 0;
                Enabled = false;
            }

            public uint GetRegister(long offset)
            {
                return Registers.TryGetValue(offset, out var value) ? value : 0;
            }

            public void SetRegister(long offset, uint value)
            {
                Registers[offset] = value;
            }

            public readonly Dictionary<long, uint> Registers =
                new Dictionary<long, uint>();
            public readonly List<byte> Data = new List<byte>();
            public uint Control;
            public uint Interrupt;
            public uint TransferSizeRegister;
            public int TransferSize;
            public int Transferred;
            public bool Enabled;
        }

        private class ReceivePacket
        {
            public ReceivePacket(uint status, byte[] data, Action completed)
            {
                Status = status;
                Data = data;
                Completed = completed;
            }

            public uint Status { get; }
            public byte[] Data { get; }
            public Action Completed { get; }
            public int Offset { get; set; }
        }

        private class HostPacket
        {
            public HostPacket(byte endpoint, byte[] data)
            {
                Endpoint = endpoint;
                Data = data;
            }

            public byte Endpoint { get; }
            public byte[] Data { get; }
            public int Offset { get; set; }
        }
    }
}
