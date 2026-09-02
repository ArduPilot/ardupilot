// Runtime peripheral removal helpers for the graphical launcher.
using System;
using System.Linq;
using System.Reflection;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;

namespace Antmicro.Renode.Core
{
    public static class AP_HotplugExtensions
    {
        public static void APHotUnplug(this Machine machine, string path,
                                       string externalName = "")
        {
            IPeripheral peripheral;
            if(!machine.TryGetByName(path, out peripheral))
            {
                throw new RecoverableException(
                    string.Format("No peripheral named {0}", path));
            }

            var emulation = EmulationManager.Instance.CurrentEmulation;
            IExternal external = null;
            if(!string.IsNullOrEmpty(externalName) &&
               !emulation.ExternalsManager.TryGetByName(
                   externalName, out external))
            {
                throw new RecoverableException(
                    string.Format("No external named {0}", externalName));
            }

            using(machine.ObtainPausedState(true))
            {
                var parents = machine.GetParentPeripherals(peripheral).ToArray();
                if(parents.Length != 1)
                {
                    throw new RecoverableException(
                        string.Format("Peripheral {0} has {1} parents", path,
                                      parents.Length));
                }

                var parent = parents[0];
                var unregister = parent.GetType().GetMethods()
                    .Where(method => method.Name == "Unregister")
                    .Where(method => method.GetParameters().Length == 1)
                    .Where(method => method.GetParameters()[0].ParameterType
                        .IsInstanceOfType(peripheral))
                    .OrderBy(method => method.GetParameters()[0].ParameterType ==
                             typeof(IPeripheral))
                    .FirstOrDefault();
                if(unregister == null)
                {
                    throw new RecoverableException(
                        string.Format("Parent of {0} cannot unregister it", path));
                }

                emulation.Connector.DisconnectFromAll(peripheral);
                try
                {
                    unregister.Invoke(parent, new object[] { peripheral });
                }
                catch(TargetInvocationException exception)
                {
                    throw new RecoverableException(
                        exception.InnerException != null
                            ? exception.InnerException.Message
                            : exception.Message);
                }

                if(external != null)
                {
                    emulation.Connector.DisconnectFromAll(external);
                    emulation.ExternalsManager.RemoveExternal(external);
                }

                var disposable = peripheral as IDisposable;
                if(disposable != null)
                {
                    disposable.Dispose();
                }
            }
        }
    }
}
