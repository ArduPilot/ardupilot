// Lockstep bridge between Renode virtual time and an ArduPilot physics
// sidecar. Sensor models obtain an atomic, machine-scoped truth snapshot
// from AP_PhysicsState so every value within a sample comes from one step.
using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Runtime.CompilerServices;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Newtonsoft.Json.Linq;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public static class AP_SensorOrientation
    {
        // ArduPilot IMU declarations describe the rotation from sensor axes
        // to body axes. Physics truth is already in body axes, so peripheral
        // models need the inverse rotation when producing raw sensor samples.
        public static float[] BodyToSensor(float[] body, byte rotation)
        {
            var xAxis = Rotate(new double[] { 1.0, 0.0, 0.0 }, rotation);
            var yAxis = Rotate(new double[] { 0.0, 1.0, 0.0 }, rotation);
            var zAxis = Rotate(new double[] { 0.0, 0.0, 1.0 }, rotation);
            return new float[] {
                (float)(xAxis[0] * body[0] + xAxis[1] * body[1] + xAxis[2] * body[2]),
                (float)(yAxis[0] * body[0] + yAxis[1] * body[1] + yAxis[2] * body[2]),
                (float)(zAxis[0] * body[0] + zAxis[1] * body[1] + zAxis[2] * body[2]),
            };
        }

        private static double[] Rotate(double[] value, byte rotation)
        {
            if(rotation <= 7)
            {
                return Yaw(value, rotation * 45.0);
            }
            if(rotation >= 8 && rotation <= 15)
            {
                return Yaw(Roll(value, 180.0), (rotation - 8) * 45.0);
            }
            if(rotation >= 16 && rotation <= 19)
            {
                return Yaw(Roll(value, 90.0), (rotation - 16) * 45.0);
            }
            if(rotation >= 20 && rotation <= 23)
            {
                return Yaw(Roll(value, 270.0), (rotation - 20) * 45.0);
            }
            switch(rotation)
            {
            case 24:
                return Pitch(value, 90.0);
            case 25:
                return Pitch(value, 270.0);
            case 26:
                return Yaw(Pitch(value, 180.0), 90.0);
            case 27:
                return Yaw(Pitch(value, 180.0), 270.0);
            case 28:
                return Pitch(Roll(value, 90.0), 90.0);
            case 29:
                return Pitch(Roll(value, 180.0), 90.0);
            case 30:
                return Pitch(Roll(value, 270.0), 90.0);
            case 31:
                return Pitch(Roll(value, 90.0), 180.0);
            case 32:
                return Pitch(Roll(value, 270.0), 180.0);
            case 33:
                return Pitch(Roll(value, 90.0), 270.0);
            case 34:
                return Pitch(Roll(value, 180.0), 270.0);
            case 35:
                return Pitch(Roll(value, 270.0), 270.0);
            case 36:
                return Yaw(Pitch(Roll(value, 90.0), 180.0), 90.0);
            case 37:
                return Yaw(Roll(value, 90.0), 270.0);
            case 38:
                return new double[] {
                    0.14303897231223747 * value[0] + 0.36877648650320383 * value[1] - 0.9184463813430871 * value[2],
                    -0.3321327777966474 * value[0] - 0.8562894214664188 * value[1] - 0.3955455025629652 * value[2],
                    -0.9323238012155122 * value[0] + 0.3616245700820924 * value[1],
                };
            case 39:
                return Pitch(value, 315.0);
            case 40:
                return Pitch(Roll(value, 90.0), 315.0);
            case 41:
                return Pitch(value, 7.0);
            case 42:
                return Roll(value, 45.0);
            case 43:
                return Roll(value, 315.0);
            default:
                return value;
            }
        }

        private static double[] Roll(double[] value, double degrees)
        {
            var radians = degrees * Math.PI / 180.0;
            var cosine = Math.Cos(radians);
            var sine = Math.Sin(radians);
            return new double[] {
                value[0],
                cosine * value[1] - sine * value[2],
                sine * value[1] + cosine * value[2],
            };
        }

        private static double[] Pitch(double[] value, double degrees)
        {
            var radians = degrees * Math.PI / 180.0;
            var cosine = Math.Cos(radians);
            var sine = Math.Sin(radians);
            return new double[] {
                cosine * value[0] + sine * value[2],
                value[1],
                -sine * value[0] + cosine * value[2],
            };
        }

        private static double[] Yaw(double[] value, double degrees)
        {
            var radians = degrees * Math.PI / 180.0;
            var cosine = Math.Cos(radians);
            var sine = Math.Sin(radians);
            return new double[] {
                cosine * value[0] - sine * value[1],
                sine * value[0] + cosine * value[1],
                value[2],
            };
        }
    }

    public static class AP_SensorNoise
    {
        public static float Pressure(AP_PhysicsTruth truth, uint sampleNumber,
            uint salt, float amplitudePa)
        {
            // Real pressure sensors never return a bit-identical stream.  The
            // barometer frontend treats two seconds without any change as a
            // failed sensor, so add small, repeatable quantisation noise at
            // the peripheral rather than contaminating physics truth.
            var sample = unchecked(sampleNumber * 1664525U + 1013904223U + salt);
            var unit = (sample & 0xFFFFU) / 32767.5f - 1.0f;
            return truth.PressurePa + unit * amplitudePa;
        }
    }

    public struct AP_PhysicsActuator
    {
        public AP_PhysicsActuator(ushort value, byte protocol, byte flags)
        {
            Value = value;
            Protocol = protocol;
            Flags = flags;
        }

        public ushort Value { get; }
        public byte Protocol { get; }
        public byte Flags { get; }

        public const int Count = 32;
        public const byte ProtocolPwm = 1;
        public const byte ProtocolDshot = 2;
        public const byte FlagValid = 1;
    }

    public interface IAP_PhysicsActuatorSource
    {
        void Sample(AP_PhysicsActuator[] actuators);
    }

    public sealed class AP_PhysicsTruth
    {
        public AP_PhysicsTruth(ulong timestampUs, uint sequence, uint flags,
            double latitudeDeg, double longitudeDeg, double altitudeM,
            double[] positionNedM, float[] quaternion, float[] gyroRadS,
            float[] specificForceMS2, float[] velocityNedMS, float airspeedMS,
            float[] magneticFieldBodyMgauss, float pressurePa,
            float temperatureK, float batteryVoltageV, float batteryCurrentA,
            float[] rpm, float[] rangefinderM)
        {
            TimestampUs = timestampUs;
            Sequence = sequence;
            Flags = flags;
            LatitudeDeg = latitudeDeg;
            LongitudeDeg = longitudeDeg;
            AltitudeM = altitudeM;
            PositionNedM = positionNedM;
            Quaternion = quaternion;
            GyroRadS = gyroRadS;
            SpecificForceMS2 = specificForceMS2;
            VelocityNedMS = velocityNedMS;
            AirspeedMS = airspeedMS;
            MagneticFieldBodyMgauss = magneticFieldBodyMgauss;
            PressurePa = pressurePa;
            TemperatureK = temperatureK;
            BatteryVoltageV = batteryVoltageV;
            BatteryCurrentA = batteryCurrentA;
            Rpm = rpm;
            RangefinderM = rangefinderM;
        }

        public ulong TimestampUs { get; }
        public uint Sequence { get; }
        public uint Flags { get; }
        public double LatitudeDeg { get; }
        public double LongitudeDeg { get; }
        public double AltitudeM { get; }
        public double[] PositionNedM { get; }
        public float[] Quaternion { get; }
        public float[] GyroRadS { get; }
        public float[] SpecificForceMS2 { get; }
        public float[] VelocityNedMS { get; }
        public float AirspeedMS { get; }
        public float[] MagneticFieldBodyMgauss { get; }
        public float PressurePa { get; }
        public float TemperatureK { get; }
        public float BatteryVoltageV { get; }
        public float BatteryCurrentA { get; }
        public float[] Rpm { get; }
        public float[] RangefinderM { get; }

        public static AP_PhysicsTruth Stationary => new AP_PhysicsTruth(
            0, 0, 0, -35.363261, 149.165230, 584.0,
            new double[] { 0.0, 0.0, 0.0 },
            new float[] { 1.0f, 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, -9.80665f },
            new float[] { 0.0f, 0.0f, 0.0f }, 0.0f,
            new float[] { 201.0f, 0.0f, 450.0f },
            94502.8f, 284.93f, 12.0f, 0.0f,
            new float[AP_PhysicsProtocol.RpmCount],
            new float[AP_PhysicsProtocol.RangefinderCount]);
    }

    public sealed class AP_PhysicsState
    {
        public static AP_PhysicsState ForMachine(IMachine machine)
        {
            return states.GetValue(machine, _ => new AP_PhysicsState());
        }

        public AP_PhysicsTruth Current
        {
            get
            {
                lock(sync)
                {
                    return current;
                }
            }
        }

        internal void Update(AP_PhysicsTruth truth)
        {
            lock(sync)
            {
                current = truth;
            }
        }

        public void RegisterActuatorSource(IAP_PhysicsActuatorSource source)
        {
            lock(sync)
            {
                if(!actuatorSources.Contains(source))
                {
                    actuatorSources.Add(source);
                }
            }
        }

        public AP_PhysicsActuator[] SampleActuators()
        {
            IAP_PhysicsActuatorSource[] sources;
            lock(sync)
            {
                sources = actuatorSources.ToArray();
            }
            var actuators = new AP_PhysicsActuator[AP_PhysicsProtocol.ActuatorCount];
            foreach(var source in sources)
            {
                source.Sample(actuators);
            }
            return actuators;
        }

        private AP_PhysicsState()
        {
            current = AP_PhysicsTruth.Stationary;
        }

        private AP_PhysicsTruth current;
        private readonly System.Collections.Generic.List<IAP_PhysicsActuatorSource> actuatorSources =
            new System.Collections.Generic.List<IAP_PhysicsActuatorSource>();
        private readonly object sync = new object();
        private static readonly ConditionalWeakTable<IMachine, AP_PhysicsState> states =
            new ConditionalWeakTable<IMachine, AP_PhysicsState>();
    }

    public class AP_Physics : IDoubleWordPeripheral, IKnownSize, IDisposable
    {
        public AP_Physics(IMachine machine)
        {
            this.machine = machine;
            state = AP_PhysicsState.ForMachine(machine);
            stepper = machine.ObtainManagedThread(
                Step, DefaultRateHz, name: "AP physics lockstep", owner: this,
                stopCondition: () => !connected);
        }

        public void Connect(int port, string model, double latitudeDeg,
            double longitudeDeg, double altitudeM, double headingDeg,
            uint rateHz = DefaultRateHz)
        {
            if(port < 1 || port > 65535)
            {
                throw new RecoverableException("physics port must be from 1 to 65535");
            }
            if(string.IsNullOrWhiteSpace(model))
            {
                throw new RecoverableException("physics model cannot be empty");
            }
            if(!IsFinite(latitudeDeg) || latitudeDeg < -90.0 || latitudeDeg > 90.0 ||
               !IsFinite(longitudeDeg) || longitudeDeg < -180.0 || longitudeDeg > 180.0 ||
               !IsFinite(altitudeM) || !IsFinite(headingDeg))
            {
                throw new RecoverableException("invalid physics location");
            }
            if(rateHz < MinimumRateHz || rateHz > MaximumRateHz)
            {
                throw new RecoverableException(string.Format(
                    "physics rate must be from {0} to {1} Hz", MinimumRateHz, MaximumRateHz));
            }

            AbortIo();
            using(machine.ObtainPausedState(true))
            {
                lock(lifecycle)
                {
                    Close(true);
                    try
                    {
                        disconnectRequested = false;
                        client = new TcpClient(AddressFamily.InterNetwork);
                        client.NoDelay = true;
                        client.ReceiveTimeout = IoTimeoutMs;
                        client.SendTimeout = IoTimeoutMs;
                        client.Connect(IPAddress.Loopback, port);
                        stream = client.GetStream();

                        SendJson(MessageType.Hello, new JObject {
                            ["role"] = "renode",
                        });
                        var hello = ReceiveJson(MessageType.HelloReply);
                        var models = hello["models"] as JArray;
                        if((string)hello["role"] != "physics" || models == null ||
                           !ContainsString(models, model))
                        {
                            throw new InvalidDataException(
                                "physics HELLO_REPLY has no matching role or model");
                        }

                        SendJson(MessageType.Configure, new JObject {
                            ["location"] = new JObject {
                                ["altitude_m"] = altitudeM,
                                ["heading_deg"] = headingDeg,
                                ["latitude_deg"] = latitudeDeg,
                                ["longitude_deg"] = longitudeDeg,
                            },
                            ["model"] = model,
                            ["rate_hz"] = rateHz,
                        });
                        var configured = ReceiveJson(MessageType.ConfigureReply);
                        if((string)configured["status"] != "configured" ||
                           (string)configured["model"] != model)
                        {
                            throw new InvalidDataException(
                                "physics CONFIGURE_REPLY did not confirm the model");
                        }

                        sequence = 0;
                        timestampUs = 0;
                        timestampRemainder = 0;
                        rate = rateHz;
                        wholePeriodUs = MicrosecondsPerSecond / rate;
                        remainderPerStep = MicrosecondsPerSecond % rate;
                        Steps = 0;
                        LastError = "";
                        stepper.Frequency = rate;
                        connected = true;
                        stepper.Start();
                        stepperStarted = true;
                        this.Log(LogLevel.Info, "physics connected to {0} on localhost:{1} at {2} Hz",
                            model, port, rate);
                    }
                    catch(Exception exception)
                    {
                        LastError = exception.Message;
                        Close(true);
                        throw new RecoverableException(string.Format(
                            "physics connection failed: {0}", LastError));
                    }
                }
            }
        }

        public void Disconnect()
        {
            AbortIo();
            using(machine.ObtainPausedState(true))
            {
                lock(lifecycle)
                {
                    Close(true);
                }
            }
        }

        public void Dispose()
        {
            AbortIo();
            lock(lifecycle)
            {
                Close(true);
                stepper.Dispose();
            }
        }

        public void Reset()
        {
            // A flight-controller reset does not reset or disconnect the
            // physical vehicle.
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case 0x00:
                return connected ? 1U : 0U;
            case 0x04:
                return Steps;
            case 0x08:
                return state.Current.Sequence;
            default:
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public bool Connected => connected;
        public uint Steps { get; private set; }
        public string LastError { get; private set; } = "";
        public long Size => 0x100;

        private void Step()
        {
            lock(lifecycle)
            {
                if(!connected)
                {
                    return;
                }
                try
                {
                    if(sequence == UInt32.MaxValue)
                    {
                        throw new InvalidDataException("physics sequence exhausted");
                    }
                    sequence++;
                    timestampUs += wholePeriodUs;
                    timestampRemainder += remainderPerStep;
                    if(timestampRemainder >= rate)
                    {
                        timestampUs++;
                        timestampRemainder -= rate;
                    }
                    SendMessage(MessageType.Step,
                        AP_PhysicsProtocol.BuildStep(
                            timestampUs, sequence, state.SampleActuators()));
                    var response = ReceiveMessage();
                    if(response.Item1 == MessageType.Error)
                    {
                        throw new InvalidDataException(ErrorText(response.Item2));
                    }
                    if(response.Item1 != MessageType.State)
                    {
                        throw new InvalidDataException(string.Format(
                            "expected STATE, received {0}", response.Item1));
                    }
                    var truth = AP_PhysicsProtocol.ParseTruth(response.Item2);
                    if(truth.Sequence != sequence || truth.TimestampUs != timestampUs)
                    {
                        throw new InvalidDataException(
                            "physics STATE sequence or timestamp does not match STEP");
                    }
                    state.Update(truth);
                    Steps++;
                }
                catch(Exception exception)
                {
                    if(!disconnectRequested)
                    {
                        LastError = exception.Message;
                        this.Log(LogLevel.Error, "physics disconnected: {0}", LastError);
                    }
                    Close(false);
                }
            }
        }

        private void AbortIo()
        {
            // Closing TcpClient is safe from another host thread and unblocks
            // an in-flight NetworkStream.Read. Do this before waiting for a
            // Renode paused state so disconnect remains responsive when a
            // sidecar has stopped replying mid-step.
            disconnectRequested = true;
            connected = false;
            var activeClient = client;
            if(activeClient != null)
            {
                activeClient.Close();
            }
        }

        private void Close(bool stopStepper)
        {
            connected = false;
            if(stopStepper && stepperStarted)
            {
                stepper.Stop();
                stepperStarted = false;
            }
            if(stream != null)
            {
                stream.Dispose();
                stream = null;
            }
            if(client != null)
            {
                client.Close();
                client = null;
            }
        }

        private void SendJson(MessageType type, JObject value)
        {
            SendMessage(type, Encoding.UTF8.GetBytes(
                value.ToString(Newtonsoft.Json.Formatting.None)));
        }

        private JObject ReceiveJson(MessageType expected)
        {
            var response = ReceiveMessage();
            if(response.Item1 == MessageType.Error)
            {
                throw new InvalidDataException(ErrorText(response.Item2));
            }
            if(response.Item1 != expected)
            {
                throw new InvalidDataException(string.Format(
                    "expected {0}, received {1}", expected, response.Item1));
            }
            try
            {
                return JObject.Parse(Encoding.UTF8.GetString(response.Item2));
            }
            catch(Exception exception)
            {
                throw new InvalidDataException("invalid JSON control payload", exception);
            }
        }

        private void SendMessage(MessageType type, byte[] payload)
        {
            if(payload.Length > AP_PhysicsProtocol.MaximumPayload)
            {
                throw new InvalidDataException("physics payload exceeds limit");
            }
            var header = new byte[EnvelopeSize];
            Buffer.BlockCopy(Magic, 0, header, 0, Magic.Length);
            AP_PhysicsProtocol.WriteUInt16(header, 4, ProtocolVersion);
            AP_PhysicsProtocol.WriteUInt16(header, 6, (ushort)type);
            AP_PhysicsProtocol.WriteUInt32(header, 8, (uint)payload.Length);
            stream.Write(header, 0, header.Length);
            stream.Write(payload, 0, payload.Length);
        }

        private Tuple<MessageType, byte[]> ReceiveMessage()
        {
            var header = ReadExact(EnvelopeSize);
            for(var i = 0; i < Magic.Length; i++)
            {
                if(header[i] != Magic[i])
                {
                    throw new InvalidDataException("invalid physics protocol magic");
                }
            }
            var version = AP_PhysicsProtocol.ReadUInt16(header, 4);
            if(version != ProtocolVersion)
            {
                throw new InvalidDataException(string.Format(
                    "unsupported physics protocol version {0}", version));
            }
            var typeValue = AP_PhysicsProtocol.ReadUInt16(header, 6);
            if(!Enum.IsDefined(typeof(MessageType), typeValue))
            {
                throw new InvalidDataException(string.Format(
                    "unknown physics message type {0}", typeValue));
            }
            var length = AP_PhysicsProtocol.ReadUInt32(header, 8);
            if(length > AP_PhysicsProtocol.MaximumPayload)
            {
                throw new InvalidDataException("physics payload exceeds limit");
            }
            return Tuple.Create((MessageType)typeValue, ReadExact((int)length));
        }

        private byte[] ReadExact(int length)
        {
            var result = new byte[length];
            var offset = 0;
            while(offset < result.Length)
            {
                var count = stream.Read(result, offset, result.Length - offset);
                if(count == 0)
                {
                    throw new EndOfStreamException("physics sidecar disconnected");
                }
                offset += count;
            }
            return result;
        }

        private static bool ContainsString(JArray values, string wanted)
        {
            foreach(var value in values)
            {
                if((string)value == wanted)
                {
                    return true;
                }
            }
            return false;
        }

        private static string ErrorText(byte[] payload)
        {
            try
            {
                var message = JObject.Parse(Encoding.UTF8.GetString(payload));
                return (string)message["error"] ?? "physics sidecar reported an error";
            }
            catch
            {
                return "physics sidecar reported an invalid error";
            }
        }

        private static bool IsFinite(double value)
        {
            return !Double.IsNaN(value) && !Double.IsInfinity(value);
        }

        private readonly IMachine machine;
        private readonly AP_PhysicsState state;
        private readonly IManagedThread stepper;
        private readonly object lifecycle = new object();
        private volatile TcpClient client;
        private NetworkStream stream;
        private volatile bool connected;
        private volatile bool disconnectRequested;
        private bool stepperStarted;
        private uint sequence;
        private uint rate;
        private ulong timestampUs;
        private uint timestampRemainder;
        private uint wholePeriodUs;
        private uint remainderPerStep;

        private const ushort ProtocolVersion = 1;
        private const int EnvelopeSize = 12;
        private const int IoTimeoutMs = 5000;
        private const uint DefaultRateHz = 400;
        private const uint MinimumRateHz = 1;
        private const uint MaximumRateHz = 10000;
        private const uint MicrosecondsPerSecond = 1000000;
        private static readonly byte[] Magic = Encoding.ASCII.GetBytes("APRP");

        private enum MessageType : ushort
        {
            Hello = 1,
            HelloReply = 2,
            Configure = 3,
            ConfigureReply = 4,
            Step = 5,
            State = 6,
            Error = 7,
        }
    }

    internal static class AP_PhysicsProtocol
    {
        public static byte[] BuildStep(ulong timestampUs, uint sequence,
            AP_PhysicsActuator[] actuators)
        {
            if(actuators == null || actuators.Length != ActuatorCount)
            {
                throw new InvalidDataException(string.Format(
                    "STEP requires {0} actuators", ActuatorCount));
            }
            var payload = new byte[StepPayloadSize];
            WriteUInt64(payload, 0, timestampUs);
            WriteUInt32(payload, 8, sequence);
            WriteUInt16(payload, 12, ActuatorCount);
            var offset = StepHeaderSize;
            foreach(var actuator in actuators)
            {
                WriteUInt16(payload, offset, actuator.Value);
                payload[offset + 2] = actuator.Protocol;
                payload[offset + 3] = actuator.Flags;
                offset += ActuatorSize;
            }
            return payload;
        }

        public static AP_PhysicsTruth ParseTruth(byte[] payload)
        {
            if(payload.Length != TruthPayloadSize)
            {
                throw new InvalidDataException(string.Format(
                    "STATE payload is {0} bytes, expected {1}",
                    payload.Length, TruthPayloadSize));
            }
            var offset = 0;
            var timestampUs = TakeUInt64(payload, ref offset);
            var sequence = TakeUInt32(payload, ref offset);
            var flags = TakeUInt32(payload, ref offset);
            var latitudeDeg = TakeDouble(payload, ref offset);
            var longitudeDeg = TakeDouble(payload, ref offset);
            var altitudeM = TakeDouble(payload, ref offset);
            var position = TakeDoubles(payload, ref offset, 3);
            var quaternion = TakeFloats(payload, ref offset, 4);
            var gyro = TakeFloats(payload, ref offset, 3);
            var specificForce = TakeFloats(payload, ref offset, 3);
            var velocity = TakeFloats(payload, ref offset, 3);
            var airspeed = TakeFloat(payload, ref offset);
            var magneticField = TakeFloats(payload, ref offset, 3);
            var pressure = TakeFloat(payload, ref offset);
            var temperature = TakeFloat(payload, ref offset);
            var batteryVoltage = TakeFloat(payload, ref offset);
            var batteryCurrent = TakeFloat(payload, ref offset);
            var rpm = TakeFloats(payload, ref offset, RpmCount);
            var rangefinder = TakeFloats(payload, ref offset, RangefinderCount);
            if(!IsFinite(latitudeDeg) || latitudeDeg < -90.0 || latitudeDeg > 90.0 ||
               !IsFinite(longitudeDeg) || longitudeDeg < -180.0 || longitudeDeg > 180.0 ||
               !IsFinite(altitudeM) || !AllFinite(position) || !AllFinite(quaternion) ||
               !AllFinite(gyro) || !AllFinite(specificForce) || !AllFinite(velocity) ||
               !IsFinite(airspeed) || !AllFinite(magneticField) || !IsFinite(pressure) ||
               !IsFinite(temperature) || !IsFinite(batteryVoltage) ||
               !IsFinite(batteryCurrent) || !AllFinite(rpm) || !AllFinite(rangefinder))
            {
                throw new InvalidDataException("STATE contains an invalid numeric value");
            }
            return new AP_PhysicsTruth(timestampUs, sequence, flags,
                latitudeDeg, longitudeDeg, altitudeM, position, quaternion,
                gyro, specificForce, velocity, airspeed, magneticField,
                pressure, temperature, batteryVoltage, batteryCurrent, rpm,
                rangefinder);
        }

        public static ushort ReadUInt16(byte[] data, int offset)
        {
            return (ushort)(data[offset] | data[offset + 1] << 8);
        }

        public static uint ReadUInt32(byte[] data, int offset)
        {
            return (uint)(data[offset] | data[offset + 1] << 8 |
                data[offset + 2] << 16 | data[offset + 3] << 24);
        }

        public static void WriteUInt16(byte[] data, int offset, ushort value)
        {
            data[offset] = (byte)value;
            data[offset + 1] = (byte)(value >> 8);
        }

        public static void WriteUInt32(byte[] data, int offset, uint value)
        {
            data[offset] = (byte)value;
            data[offset + 1] = (byte)(value >> 8);
            data[offset + 2] = (byte)(value >> 16);
            data[offset + 3] = (byte)(value >> 24);
        }

        private static ulong ReadUInt64(byte[] data, int offset)
        {
            return ReadUInt32(data, offset) |
                (ulong)ReadUInt32(data, offset + 4) << 32;
        }

        private static void WriteUInt64(byte[] data, int offset, ulong value)
        {
            WriteUInt32(data, offset, (uint)value);
            WriteUInt32(data, offset + 4, (uint)(value >> 32));
        }

        private static uint TakeUInt32(byte[] data, ref int offset)
        {
            var result = ReadUInt32(data, offset);
            offset += 4;
            return result;
        }

        private static ulong TakeUInt64(byte[] data, ref int offset)
        {
            var result = ReadUInt64(data, offset);
            offset += 8;
            return result;
        }

        private static double TakeDouble(byte[] data, ref int offset)
        {
            var result = BitConverter.Int64BitsToDouble(
                unchecked((long)ReadUInt64(data, offset)));
            offset += 8;
            return result;
        }

        private static float TakeFloat(byte[] data, ref int offset)
        {
            var bytes = BitConverter.GetBytes(ReadUInt32(data, offset));
            var result = BitConverter.ToSingle(bytes, 0);
            offset += 4;
            return result;
        }

        private static double[] TakeDoubles(byte[] data, ref int offset, int count)
        {
            var result = new double[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = TakeDouble(data, ref offset);
            }
            return result;
        }

        private static float[] TakeFloats(byte[] data, ref int offset, int count)
        {
            var result = new float[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = TakeFloat(data, ref offset);
            }
            return result;
        }

        private static bool AllFinite(double[] values)
        {
            foreach(var value in values)
            {
                if(!IsFinite(value))
                {
                    return false;
                }
            }
            return true;
        }

        private static bool AllFinite(float[] values)
        {
            foreach(var value in values)
            {
                if(!IsFinite(value))
                {
                    return false;
                }
            }
            return true;
        }

        private static bool IsFinite(double value)
        {
            return !Double.IsNaN(value) && !Double.IsInfinity(value);
        }

        private static bool IsFinite(float value)
        {
            return !Single.IsNaN(value) && !Single.IsInfinity(value);
        }

        public const int MaximumPayload = 1024 * 1024;
        public const ushort ActuatorCount = AP_PhysicsActuator.Count;
        public const int RpmCount = 32;
        public const int RangefinderCount = 10;
        private const int StepHeaderSize = 16;
        private const int ActuatorSize = 4;
        private const int StepPayloadSize = StepHeaderSize + ActuatorCount * ActuatorSize;
        private const int TruthPayloadSize = 16 + 6 * 8 + 63 * 4;
    }
}
