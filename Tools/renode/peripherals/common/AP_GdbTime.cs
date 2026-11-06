// Keep real-time pacing across GDB halts. Renode's host stopwatch keeps
// running while GDB holds the CPU, so after a long halt virtual time lags
// host time and the emulation runs unthrottled until it catches up.
using System.Diagnostics;
using System.Reflection;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Core
{
    public static class AP_GdbTimeExtensions
    {
        // Discard host time that has passed since the last time update.
        public static void APResyncHostTime(this Emulation emulation)
        {
            Resync(emulation.MasterTimeSource);
            foreach(var machine in emulation.Machines)
            {
                Resync(machine.LocalTimeSource as TimeSourceBase);
            }
        }

        private static void Resync(TimeSourceBase source)
        {
            if(source == null)
            {
                return;
            }
            var hostTicks = Field("hostTicksElapsed").GetValue(source);
            var stopwatch = (Stopwatch)Field("stopwatch").GetValue(source);
            lock(hostTicks)
            {
                Field("elapsedAtLastUpdate").SetValue(
                    source, TimeInterval.FromTimeSpan(stopwatch.Elapsed));
            }
        }

        private static FieldInfo Field(string name)
        {
            var field = typeof(TimeSourceBase).GetField(
                name, BindingFlags.Instance | BindingFlags.NonPublic);
            if(field == null)
            {
                throw new RecoverableException(
                    "this Renode has no TimeSourceBase." + name);
            }
            return field;
        }
    }
}
