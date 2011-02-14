using System;

namespace ArducopterConfigurator
{

    public interface ISupportsExternalInvokedInpc
    {
        void FirePropertyChanged(string propName);
    }


    public interface ISupportsPropertyPopulation : ISupportsExternalInvokedInpc
    {
        string [] PropsInUpdateOrder { get;}
    }

    /// <summary>
    /// Helper class that takes an update from the APM and writes to properties of an object using reflection
    /// </summary>
    /// <remarks>
    /// The APM gives us a stream of values .eg 4.0,5.2,3.4 etc
    /// They are in a certain known order, e.g pitch P, pitch I, pitch D... etc
    /// 
    /// instead of having specific value assignment logic in every class to pick
    /// apart the update and populate properties, this guy will Set the properties
    /// in the correct order. All the interested class needs to provide is the
    /// list of property names in the same order as the update from the APM
    /// </remarks>
    public static class PropertyHelper
    {
        // Common method for creating the update data
        // sentence sent to APM is the commandChar followed by the property
        // vals in the correct order, seperated by semicolons
        public static string ComposePropValuesWithCommand(ISupportsPropertyPopulation obj, string commandChar)
        {
            var strings = new string[obj.PropsInUpdateOrder.Length];
            for (int i = 0; i < obj.PropsInUpdateOrder.Length; i++)
            {
                var prop = obj.GetType().GetProperty(obj.PropsInUpdateOrder[i]);

                if (prop.PropertyType == typeof(bool))
                    strings[i] = ((bool)prop.GetValue(obj, null)) ? "1" : "0";
                else
                    strings[i] = prop.GetValue(obj, null).ToString();

            }

            return commandChar + string.Join(";", strings);
        }

        // Common method for populating properties, using a hardcoded 
        // property update order, and reflection to get the property type
        public static void PopulatePropsFromUpdate(ISupportsPropertyPopulation obj,string strRx, bool fireInpc)
        {
            var strs = strRx.Split(',');

            if (obj.PropsInUpdateOrder.Length != strs.Length)
            {
                Console.WriteLine("Processing update with " + strs.Length
                                  + " values, but have " + obj.PropsInUpdateOrder.Length
                                  + " properties to populate. Ignoring this update");
                return;
            }

            for (int i = 0; i < obj.PropsInUpdateOrder.Length; i++)
            {
                var prop = obj.GetType().GetProperty(obj.PropsInUpdateOrder[i]);
                var s = strs[i];
                object value = null;

                if (prop == null)
                {
                    Console.WriteLine("Trying to set non existant property: " + obj.PropsInUpdateOrder[i]);
                    break;
                }

                if (prop.PropertyType == typeof(float))
                {
                    float val;
                    if (!float.TryParse(s, out val))
                    {
                        Console.WriteLine("Error parsing float: {0}, VM: {1}" + s, "TODO");
                        break;
                    }
                    value = val;
                }
                if (prop.PropertyType == typeof(bool))
                {
                    float val;
                    if (!float.TryParse(s, out val))
                    {
                        Console.WriteLine("Error parsing float (bool): {0}, VM: {1}" + s, "TODO");
                        break;
                    }
                    value = val != 0.0;
                }

                if (prop.PropertyType == typeof(int))
                {
                    int val;
                    if (!int.TryParse(s, out val))
                    {
                        Console.WriteLine("Error parsing int:{0}, VM: {1}" + s, "TODO");
                        break;
                    }
                    value = val;
                }

                prop.SetValue(obj, value, null);
                
                if (fireInpc)
                    obj.FirePropertyChanged(obj.PropsInUpdateOrder[i]);
            }
        }

    }

  


    public abstract class VmBase : NotifyProperyChangedBase
    {
        protected string[] PropsInUpdateOrder;

        // Common method for creating the update data
        // sentence sent to APM is the commandChar followed by the property
        // vals in the correct order, seperated by semicolons
        protected string ComposePropsWithCommand(string commandChar)
        {
            var strings = new string[PropsInUpdateOrder.Length];
            for (int i = 0; i < PropsInUpdateOrder.Length; i++)
            {
                var prop = this.GetType().GetProperty(PropsInUpdateOrder[i]);

                if (prop.PropertyType == typeof(bool))
                    strings[i] = ((bool)prop.GetValue(this, null)) ? "1" : "0";
                else
                    strings[i] = prop.GetValue(this, null).ToString();

            }

            return commandChar + string.Join(";", strings);
        }

        // Common method for populating properties, using a hardcoded 
        // property update order, and reflection to get the property type
        protected void PopulatePropsFromUpdate(string strRx, bool fireInpc)
        {

            var strs = strRx.Split(',');

            if (PropsInUpdateOrder.Length != strs.Length)
            {
                Console.WriteLine("Processing update with " + strs.Length
                                  + " values, but have " + PropsInUpdateOrder.Length
                                  + " properties to populate. Ignoring this update");
                return;
            }

            for (int i = 0; i < PropsInUpdateOrder.Length; i++)
            {
                var prop = this.GetType().GetProperty(PropsInUpdateOrder[i]);
                var s = strs[i];
                object value = null;

                if (prop == null)
                {
                    Console.WriteLine("Trying to set non existant property: " + PropsInUpdateOrder[i]);
                    break;
                }

                if (prop.PropertyType == typeof(float))
                {
                    float val;
                    if (!float.TryParse(s, out val))
                    {
                        Console.WriteLine("Error parsing float: {0}, VM: {1}" + s, "TODO");
                        break;
                    }
                    value = val;
                }
                if (prop.PropertyType == typeof(bool))
                {
                    float val;
                    if (!float.TryParse(s, out val))
                    {
                        Console.WriteLine("Error parsing float (bool): {0}, VM: {1}" + s, "TODO");
                        break;
                    }
                    value = val != 0.0;
                }

                if (prop.PropertyType == typeof(int))
                {
                    int val;
                    if (!int.TryParse(s, out val))
                    {
                        Console.WriteLine("Error parsing int:{0}, VM: {1}" + s, "TODO");
                        break;
                    }
                    value = val;
                }

                prop.SetValue(this, value, null);

                if (fireInpc)
                    FirePropertyChanged(PropsInUpdateOrder[i]);
            }
        }

    }
}